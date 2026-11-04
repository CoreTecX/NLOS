"""Offline regression for current-package selection and module integrity failures."""
from pathlib import Path
import argparse
import contextlib
import copy
import importlib.util
import io
import json
import os
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
REPO = Path(__file__).resolve().parents[2]


def import_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CHECK = import_file('workspace_inspection', REPO / 'build/scripts/workspace_check.py')
BUILD = import_file('full_model_entry', REPO / 'build/scripts/full_model.py')


class EntryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ps_module_entry_', dir='D:/Temp' if os.name=='nt' else None)
        self.root = Path(self.temp.name)
        self.config = dict(software_revision='current-software', hardware_release='current-hardware',
                           layout_abi='0x32525001', frozen_release='packages/frozen',
                           default_hardware_profile='slow', recommended_performance_profile='fast',
                           sources=[dict(repository='src/core/entry.cpp', release='src/entry.cpp')],
                           python_model_sources=[])
        self.registry = dict(schema_version=1, software_revision='current-software', hardware_release='current-hardware',
                             layout_abi='0x32525001', physical_board_test=False, candidate_implementation=False,
                             module_count=1, release_packages={'performance':'packages/performance','fallback':'packages/fallback'},
                             v3_counts={'离线通过':1}, modules=[dict(path='src/core',readme='src/core/README.md',
                             entry_points=['src/core/entry.cpp'],evidence=['evidence.json'],dependencies=[],
                             preservation_reason='active',boundary='offline only',classification='active')])
        self.write('src/core/README.md','fixture')
        self.write('src/core/entry.cpp','int value = 1;\n')
        self.write('evidence.json',{})
        self.write('report/optimization_plan_v3/tasks.csv','任务ID,状态\nP0.1,离线通过\n')
        for name, profile in [('performance','fast'),('fallback','slow')]:
            self.write('packages/'+name+'/src/entry.cpp','int value = 1;\n')
            self.write('packages/'+name+'/release_manifest.json',dict(software_revision='current-software',
                       hardware_release='current-hardware',layout_version=0x32525001,hardware_profile=profile,physical_board_test=False))
        self.save()

    def tearDown(self):
        self.temp.cleanup()

    def write(self, relative, content):
        path=self.root/relative;path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(content if isinstance(content,str) else json.dumps(content,ensure_ascii=False),encoding='utf-8')

    def save(self):
        self.write('configs/full_model_m8.json',self.config)
        self.write('configs/workspace_modules.json',self.registry)

    def rejected(self):
        self.save()
        try:
            result=CHECK.inspect(self.root)
        except (ValueError,KeyError,OSError,TypeError):
            return
        self.assertFalse(result['passed'])

    def test_valid_registry_and_sources(self):
        self.assertTrue(CHECK.inspect(self.root)['passed'])

    def test_default_checks_both_current_packages(self):
        paths=BUILD.release_check_paths(self.root,self.config)
        self.assertEqual(paths,[self.root/'packages/performance',self.root/'packages/fallback'])
        self.assertNotIn(self.root/'packages/frozen',paths)

    def test_frozen_requires_explicit_selection(self):
        self.assertEqual(BUILD.release_check_paths(self.root,self.config,package='frozen'),[self.root/'packages/frozen'])

    def test_explicit_package_path_is_preserved(self):
        self.assertEqual(BUILD.release_check_paths(self.root,self.config,self.root/'custom'),[self.root/'custom'])
        with self.assertRaises(ValueError):
            BUILD.release_check_paths(self.root,self.config,self.root/'custom',package='fallback')

    def test_swapped_package_profile_rejected(self):
        p=self.root/'packages/performance/release_manifest.json'
        data=json.loads(p.read_text('utf-8'));data['hardware_profile']='slow';self.write(p,data)
        with self.assertRaises(ValueError):
            BUILD.release_check_paths(self.root,self.config)
        self.rejected()

    def test_stale_software_rejected(self):
        self.registry['software_revision']='old';self.rejected()
        with self.assertRaises(ValueError):
            BUILD.release_check_paths(self.root,self.config)

    def test_missing_module_entry_rejected(self):
        self.registry['modules'][0]['entry_points'].append('src/core/missing.cpp');self.rejected()

    def test_duplicate_module_rejected(self):
        self.registry['modules'].append(copy.deepcopy(self.registry['modules'][0]));self.registry['module_count']=2;self.rejected()

    def test_missing_evidence_rejected(self):
        self.registry['modules'][0]['evidence']=['missing.json'];self.rejected()

    def test_unknown_dependency_rejected(self):
        self.registry['modules'][0]['dependencies']=['src/absent'];self.rejected()

    def test_source_drift_rejected(self):
        self.write('src/core/entry.cpp','int value = 2;\n');self.rejected()

    def test_unassigned_source_rejected(self):
        self.write('other/entry.cpp','int value = 1;\n');self.config['sources'][0]['repository']='other/entry.cpp';self.rejected()

    def test_reference_drift_rejected(self):
        self.write('src/core/reference.py','reference=1\n')
        self.config['python_model_sources']=[dict(repository='src/core/reference.py',sha256='0'*64)];self.rejected()

    def test_false_board_claim_rejected(self):
        self.registry['physical_board_test']=True;self.rejected()

    def test_false_candidate_claim_rejected(self):
        self.registry['candidate_implementation']=True;self.rejected()

    def test_task_count_drift_rejected(self):
        self.registry['v3_counts']={'离线通过':2};self.rejected()

    def test_repository_path_escape_rejected(self):
        self.registry['modules'][0]['entry_points']=['../outside.cpp'];self.rejected()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(EntryTests))
    summary=dict(passed=result.wasSuccessful(),cases=result.testsRun,failures=len(result.failures),errors=len(result.errors),physical_board_test=False)
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        with args.report.open('x',encoding='utf-8') as stream:
            json.dump(summary,stream,indent=2);stream.write('\n')
    print(json.dumps(summary))
    return 0 if summary['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())

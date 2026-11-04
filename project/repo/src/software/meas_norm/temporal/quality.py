"""Registered diagnostic SNR from a fixed reference; never a probability."""
import numpy as np
from contracts import require,sha


class QualityEstimator:
    def __init__(self,reference,stable_mask,policy):
        self.reference=np.frombuffer(np.asarray(reference,np.float64).tobytes(),np.float64).reshape(126,126,3)
        self.stable=np.frombuffer(np.asarray(stable_mask,bool).tobytes(),bool).reshape(126,126)
        self.policy=dict(policy)
        require(self.stable.sum()>=100 and (~self.stable).sum()>=100,'QUALITY_REGION','Independent noise and signal regions required')
        for k in ('normalizer','minimum_noise_sigma','usable_snr_db','max_noise_sigma','max_illumination_shift'):
            require(np.isfinite(policy[k]) and policy[k]>0,'QUALITY_POLICY','Invalid '+k)
        require(policy['scope'] in ('SYNTHETIC_TEST_ONLY','REAL_CALIBRATED'),'QUALITY_SCOPE','Missing evidence scope')

    def evaluate(self,m,valid=True):
        base=dict(method='STABLE_REGION_MAD_FIXED_REFERENCE_V1',score_is_probability=False,probability=None,
                  scope=self.policy['scope'],physical_quality_accepted=False,valid=False,snr_db=None,
                  noise_sigma=None,signal_rms=None,quality='UNKNOWN',reason=None)
        if not valid:return dict(base,quality='INVALID',reason='INPUT_INVALID')
        if m.k!=1:return dict(base,quality='INVALID',reason='QUALITY_REQUIRES_SINGLE_FRAME')
        if self.policy['scope']=='SYNTHETIC_TEST_ONLY' and m.source_kind!='SYNTHETIC':return dict(base,reason='REAL_QUALITY_UNREGISTERED')
        residual=((m.rgb-self.reference)/self.policy['normalizer']).mean(axis=2)
        shift=float(np.median(residual[self.stable]))
        if abs(shift)>self.policy['max_illumination_shift']:return dict(base,quality='INVALID',reason='ILLUMINATION_OUT_OF_RANGE')
        residual=residual-shift;noise=float(np.median(np.abs(residual[self.stable]))/.6744897501960817)
        noise=max(noise,self.policy['minimum_noise_sigma'])
        power=float(np.mean(residual[~self.stable]**2));signal=float(np.sqrt(max(0.,power-noise*noise)))
        if noise>self.policy['max_noise_sigma']:return dict(base,quality='INVALID',reason='NOISE_OUT_OF_RANGE',noise_sigma=noise,signal_rms=signal)
        snr=20*float(np.log10(signal/noise)) if signal>0 else None
        quality='UNOBSERVABLE' if snr is None else 'USABLE' if snr>=self.policy['usable_snr_db'] else 'LOW_SIGNAL'
        return dict(base,valid=True,quality=quality,snr_db=snr,noise_sigma=noise,signal_rms=signal,
                    reason='NO_DETECTABLE_SIGNAL' if snr is None else None,illumination_shift=shift)


def reconstruction_quality(result):
    out=result.get('output',{})
    return dict(numeric_state='PASSED' if result.get('valid_numeric') else 'FAILED',
        reference_state='PASSED' if out.get('reference_acceptance') is True else 'NOT_AVAILABLE' if out.get('reference_acceptance') is None else 'FAILED',
        context_current=result.get('publish_allowed',False),freshness=result.get('freshness','UNKNOWN'),
        physical_quality_accepted=False,probability=None)

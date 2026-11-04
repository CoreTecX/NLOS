# 交付格式

完整工程 nlos_project_engineering 包含97例输入、模型、参考、板端验证产物及全部工程模块，供仓库分发，使用Git LFS管理大文件。

独立源码附件 nlos_source_code.rar 按RAR格式、严格小于200,000,000字节检查；保留全部源码、报告、技能和验证产物，仅提供三组匹配输入。完整97例输入在完整工程中。两个包各有独立manifest，不能混用。

所有目录名及文件名为英文ASCII，无中文和空格；README.md为官方模板给定的大小写例外，其余小写。压缩包不嵌套旧压缩包，不含实时日志和个人操作流水。

本地完整性检查：python tools/verify_package.py；结构、命名与输入检查：python tools/check_submission.py。大模型协作说明已经团队审核确认，技术指标按所附证据核对。

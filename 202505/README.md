# 202505

## 20250501

修复软件包构建问题：lanelet2

参考[https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/211](https://gitee.com/link?target=https%3A%2F%2Fgithub.com%2Ffzi-forschungszentrum-informatik%2FLanelet2%2Fissues%2F211) 添加补丁

pr链接：https://gitee.com/src-openeuler/lanelet2/pulls/10
eulermaker链接：https://eulermaker.compass-ci.openeuler.openatom.cn/project/build?osProject=zhouyu

**未合并**

## 20250502

修复软件包构建问题：zenoh-plugin-dds

1.修改了 cyclors 库中的 build.rs，增加了自定义 ParseCallbacks，对匿名枚举及特殊字符名称进行合法化处理，避免 bindgen panic。
2.添加补丁修改 Cargo.toml，使其依赖指向本地修改后的 cyclors 库，确保构建时使用修正后的代码。

pr链接：https://gitee.com/src-openeuler/zenoh-plugin-dds/pulls/7
EulerMaker链接：https://eulermaker.compass-ci.openeuler.openatom.cn/package/overview?osProject=zhouyu&packageName=zenoh-plugin-dds

**未合并**
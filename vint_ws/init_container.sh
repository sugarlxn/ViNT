#!/bin/bash
echo "container init..."
sed -i "s@http://.*archive.ubuntu.com@http://repo.huaweicloud.com@g" /etc/apt/sources.list
sed -i "s@http://.*security.ubuntu.com@http://repo.huaweicloud.com@g" /etc/apt/sources.list
apt-get update

#pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple pip3 -U # 将 pip 升级到最新版本
#pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

#pip3 install jupyter d2l -i https://pypi.tuna.tsinghua.edu.cn/simple
#pip3 install torch==1.8.2+cu111 torchvision==0.9.2+cu111 torchaudio==0.8.2 -f https://download.pytorch.org/whl/lts/1.8/torch_lts.html

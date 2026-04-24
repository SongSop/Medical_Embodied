
sudo apt install tmux


让 python cmd 指向 /usr/bin/python3:

sudo apt install python-is-python3
sudo apt install -y portaudio19-dev

装 miniconda

wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

bash ~/Miniconda3-latest-Linux-x86_64.sh

从 environment.yml 中创造环境
conda env create -f environment.yml

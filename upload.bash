#!/bin/bash
source ~/anaconda3/bin/activate open3d
cd dist
rm -rf *
cd ..
python modify_version.py
python3 setup.py sdist bdist_wheel
twine upload dist/*

from setuptools import setup
 
setup(
    name='pc_utils', #包名
    version='1.0',#版本
    description='an util for processing point cloud',#描述
    author='Grapymage',#作者
    author_email='18013933973@163.com',#作者邮箱
    packages=['pc_utils'],#需要封装的所有包
    requires=['open3d','pclpy'],
    python_requires= ">=3.8"
   )

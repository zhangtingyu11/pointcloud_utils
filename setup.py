import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="pointcloud_utils",
    version="0.0.28",
    author="Grapymage",
    author_email="18013933973@163.com",
    description="A pkg about pointcloud",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/zhangtingyu11/pointcloud_utils",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)

import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="img-shutter-face", 
    version="0.1a0",
    author="Marynel Vazquez",
    author_email="marynel.vazquez@yale.edu",
    description="A simple robot face",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://gitlab.com/interactive-machines/shutter/shutter-face",
    packages=['shutter_face'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    package_data={'': [ 
        'shutter_face/data/Angry_Overlay.svg',
        'shutter_face/data/Bored_Overlay.svg',
        'shutter_face/data/Determined_Overlay.svg',
        'shutter_face/data/Happy_Overlay_2.svg',
        'shutter_face/data/Happy_Overlay.svg',
        'shutter_face/data/Sad_Overlay.svg',
        'shutter_face/data/Surprised_Overlay.svg' 
    ]},
    install_requires=['PySide2==5.15.0', 'numpy>=1.18.0'],
)

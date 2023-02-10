from setuptools import setup

setup(name='ipp_pkg',
      version='2.0',
      description='Adaptive approach to underwater acoustic source localization and tracking',
      url='https://github.com/andreatitti97/ros_simulation_ws',
      author='Andrea Tiranti',
      author_email='andrea.tiranti97@gmail.com',
      license='UNIGE',
      packages=['ipp_pkg'],
      install_requires=[
          'numpy',
          'scipy',
          'theano',
          'pybnb'
      ],
      zip_safe=False)


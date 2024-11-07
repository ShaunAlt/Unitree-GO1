from setuptools import setup

setup(
    name = 'go1_ros_model',
    version = '1.0.0',
    description = 'Python Object Model for Unitree GO1 Robot High-Level Control',
    author = 'Shaun Altmann',
    author_email = 'shaun.altmann@deakin.edu.au',
    maintainer = 'Shaun Altmann',
    maintainer_email = 'shaun.altmann@deakin.edu.au',
    url = 'https://github.com/ShaunAlt/Unitree-GO1',
    packages = ['go1_ros_model'],
    data_files = [],
    install_requires = [
        'setuptools',
    ],
    zip_safe = True,
    license = 'TODO: License declaration',
    tests_require = ['pytest'],
    entry_points = {
        'console_scripts': [],
    },
)
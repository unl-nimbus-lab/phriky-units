#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup

with open('README.rst') as readme_file:
    readme = readme_file.read()

with open('HISTORY.rst') as history_file:
    history = history_file.read()

requirements = [
    'Click>=6.0',
    # TODO: put package requirements here
    'networkx'
]

test_requirements = [
    # TODO: put package test requirements here
]

setup(
    name='phriky_units',
    version='0.1.0a26',
    description="Physical unit static analysis tool for C++ +",
    long_description=readme + '\n\n' + history,
    author="John-Paul Ore",
    author_email='jore@cse.unl.edu',
    url='https://github.com/jpwco/phriky_units',
    packages=[
        'phriky_units',
    ],
    package_data={'phriky_units': ['resources/cppcheck/std.cfg']},
    package_dir={'phriky_units':
                 'phriky_units'},
    entry_points={
        'console_scripts': [
            'phriky_units=phriky_units.phriky_units:main'
        ]
    },
    include_package_data=True,
    install_requires=requirements,
    license="MIT license",
    zip_safe=False,
    keywords='phriky_units',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 2.7',
        # 'Programming Language :: Python :: 3',
        # 'Programming Language :: Python :: 3.3',
        # 'Programming Language :: Python :: 3.4',
        # 'Programming Language :: Python :: 3.5',
    ],
    test_suite='tests',
    tests_require=test_requirements
)

#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    packages=[
        "expert_data_rule",
        "expert_data_rule.mapping",
        "expert_data_rule.perception",
        "expert_data_rule.policy",
        "expert_data_rule.runtime",
    ],
    package_dir={"": "."},
))

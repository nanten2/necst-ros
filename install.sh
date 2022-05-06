#!/bin/bash


# Define installer for additional package Poetry cannot handle
install_additional_packages(){
    pip3 install -qU git+https://github.com/ars096/pyinterface2.git; sys_platform == 'linux'
}

# Check if poetry command available or not
if ! type poetry > /dev/null 2>&1
then
    # Install poetry
    echo -e "\033[46mInstalling poetry\033[0m"
    curl -sSL https://install.python-poetry.org | python3 -

    if type poetry > /dev/null 2>&1
    then
        echo -e "\033[46;1mPoetry successfully installed\033[0m"
    else
        echo -e "\033[41;1mInstallation failed\033[0m"
        exit 1
    fi
fi


# Change-directory to the current package
# *In case this script is executed from other directory, pyproject.toml won't be found
package_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
cd ${package_dir}


# To manage global python environment
poetry config virtualenvs.create false --local


# If the package doesn't contain pyproject.toml, create it
if ! poetry check > /dev/null 2>&1
then
    echo -e "\033[46mInitialize virtual environment\033[0m"
    poetry init
    echo -e "\033[46mVirtual environment created\033[0m"
else
    echo -e "\033[46mValid pyproject.toml exists\033[0m"
fi


# Try poetry install command
if poetry install
then
    install_additional_packages()
    echo -e "\033[46;1m===========================================\033[0m"
    echo -e "\033[46;1m= Successfully installed the dependencies =\033[0m"
    echo -e "\033[46;1m===========================================\033[0m"
else
    echo -e "\033[46;1m======================================\033[0m"
    echo -e "\033[46;1m= Installation failed, trying update =\033[0m"
    echo -e "\033[46;1m======================================\033[0m"

    # If poetry install failed, run poetry update
    if poetry update
    then
        install_additional_packages()
        echo -e "\033[46;1m=========================================\033[0m"
        echo -e "\033[46;1m= Successfully updated the dependencies =\033[0m"
        echo -e "\033[46;1m=========================================\033[0m"
    else
        echo -e "\033[41;1m=========================================\033[0m"
        echo -e "\033[41;1m= poetry failed to resolve dependencies =\033[0m"
        echo -e "\033[41;1m=========================================\033[0m"
    fi
fi


# For safety
poetry config virtualenvs.create true --local


#!/bin/bash


# Get path to shell run-command file (i.e. ~/.bashrc, ~/.zshrc, etc.)
IFS='/' read -r -a array <<< "$SHELL"
rc_path="$HOME/.${array[-1]}rc"


# Check if poetry command available or not
if ! type poetry > /dev/null 2>&1
then
    # Install poetry
    echo "Installing poetry"
    curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python3 -

    # Set path to poetry command if not done
    if grep '.poetry/bin' ${rc_path} > /dev/null 2>&1
    then
        echo -e "\033[46mPoetry path has already been set\033[0m"
    else
        echo -e "\033[46mSetting path in ${rc_path}\033[0m"
        echo '' >> ${rc_path}
        echo '# poetry' >> ${rc_path}
        echo 'export PATH="$HOME/.poetry/bin:$PATH"' >> ${rc_path}
        echo -e "\033[46;1mPoetry successfully installed\033[0m"
        export PATH="$HOME/.poetry/bin:$PATH"
        echo -e "\033[46mRestart the shell to use poetry\033[0m"
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
        echo -e "\033[46;1m=========================================\033[0m"
        echo -e "\033[46;1m= Successfully updated the dependencies =\033[0m"
        echo -e "\033[46;1m=========================================\033[0m"
    else
        echo -e "\033[46;1m=========================================\033[0m"
	echo -e "\033[46;1m= poetry failed to resolve dependencies =\033[0m"
        echo -e "\033[46;1m=========================================\033[0m"
    fi
fi


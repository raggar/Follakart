# Graduates Of Stack Overflow

## General Notes

### Pip

Pip is the reference Python package manager. It’s used to install and update packages. You’ll need to make sure you have the latest version of pip installed.

`py -m pip install --upgrade pip`

To install packages run

`pip install package-name`

### Virtual Env

Virtualenv is used to manage Python packages for different projects. Using virtualenv allows you to avoid installing Python packages globally which could break system tools or other projects. You can create a virtual enviornment by running the following commands:

Mac:
`python3 -m venv env`

Windows:
`py -m venv env`

To then activate your virtual enviornment run

Mac:
`source env/bin/activate`

Windows:
`.\env\Scripts\activate`

To leave the virtual enviornment type `deactivate` in your terminal/command prompt

### Requirements.txt

A requirements.txt file acts as a package.json file and basically lists each dependency used in the project. Thus, when pulling from master ensure that you run the following command to update your dependencies

`pip install -r requirements.txt`

Make sure that when you install new project packages that they are added to the requirements.txt file (should be kept at the root directory). Use `pip freeze` to view a list of packages.

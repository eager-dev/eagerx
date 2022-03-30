**********************
Contributing to EAGERx
**********************

If you are interested in contributing to EAGERx, your contributions will fall
into three categories: 

1. You want to propose a new Feature and implement it

    - Create an issue about your intended feature, and we shall discuss the design and
    implementation. Once we agree that the plan looks good, go ahead and implement it.

2. You want to implement a feature or bug-fix for an outstanding issue

    - Look at the outstanding issues here: https://github.com/eager-dev/eagerx/issues
    - Pick an issue or feature and comment on the task that you want to work on this feature.
    - If you need more context on a particular issue, please ask and we shall provide.

3. You want to create a new EAGERx package

    - Check out the template here: https://github.com/eager-dev/eagerx_template

Once you finish implementing a feature or bug-fix, please send a Pull Request


If you are not familiar with creating a Pull Request, here are some guides:

- http://stackoverflow.com/questions/14680711/how-to-do-a-github-pull-request
- https://help.github.com/articles/creating-a-pull-request/


Developing EAGERx
#################

To develop EAGERx on your machine, here are some tips:

1. Clone a copy of Stable-Baselines3 from source:

.. code-block:: bash

  git clone https://github.com/eager-dev/eagerx.git
  cd eagerx/

2. Install [Poetry](https://python-poetry.org/docs/) if you haven't done so yet:

.. code-block:: bash

  curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python -

3. Install EAGERx:

.. code-block:: bash

  poetry install

Codestyle
#########

We are using `black codestyle <https://github.com/psf/black>`_ (max line length of 127 characters) together with `isort <https://github.com/timothycrosley/isort>`_ to sort the imports.

**Please run *make codestyle* ** to reformat your code. You can check the codestyle using *make check-codestyle* and *make lint*.

Please document each function/method and `type <https://google.github.io/pytype/user_guide.html>`_ them using the following template:

::

  def my_function(arg1: type1, arg2: type2) -> returntype:
      """
      Short description of the function.

      :param arg1: describe what is arg1
      :param arg2: describe what is arg2
      :return: describe what is returned
      """
      ...
      return my_variable

Pull Request (PR)
#################

Before proposing a PR, please open an issue, where the feature will be discussed. This prevent from duplicated PR to be proposed and also ease the code review process.

Each PR need to be reviewed and accepted by at least one of the maintainers (@bheijden, @jelledouwe).
A PR must pass the Continuous Integration tests to be merged with the master branch.


Tests
#####

All new features must add tests in the *tests/* folder ensuring that everything works fine.
We use `pytest <https://pytest.org/>`_.
Also, when a bug fix is proposed, tests should be added to avoid regression.

To run tests with *pytest*:

:: code-block:: bash

  make pytest

Codestyle check with *black* and *flake8*:

:: code-block:: bash

  make check-codestyle
  make lint

Build the documentation:

:: code-block:: bash

  make doc

Check documentation spelling (you need to install *sphinxcontrib.spelling* package for that):

:: code-block:: bash

  make spelling

Semantic Pull Request and Documentation
#######################################

Please make sure that you use `semantic commit messages <https://github.com/zeke/semantic-pull-requests>`_ and add documentation if needed.
For example, for committing a fix, your commit message should start with *fix: *, for features with *feat: * and for breaking changes *BREAKING CHANGE: *.
You should add your username in the commit message for each fix, feature or breaking change.
The docs can be built as follows.

Make sure EAGERx is installed:

:: code-block:: bash

  cd [eagerx_root]
  poetry install

Activate the Poetry environment:

:: code-block:: bash

  poetry shell

Build the docs:

:: code-block:: bash

  make doc

Credits: this contributing guide is based on the one from `Stable Baselines3 <https://github.com/DLR-RM/stable-baselines3>`_ which in turn is based on the one from `PyTorch <https://github.com/pytorch/pytorch/>`_.

Template
########

We will start by creating a new repository for this Python package, using `the template that is available here <https://github.com/eager-dev/eagerx_template>`_.

.. figure:: figures/eagerx_template.png
    :width: 465px
    :align: center
    :height: 338px
    :alt: alternate text
    :figclass: align-center

    Screenshot of the EAGERx template package on Github.

As you can see, this template repository already contains some folders and files.
The main benefit of using this template, is that it facilitates to perform continuous integration and provides a clear code structure.
Since the package is just a Python package in the end, any other Python package structure could be used.
However, using this template makes it possible to add your package to the `EAGERx packages repository <https://github.com/eager-dev/eagerx_packages>`_ and thereby making it available for others.

In our case, we create a new repository called eagerx_ode using this template.
Since we want to create a package named eagerx_ode and not eagerx_template, we do the following:

* Rename the folder eagerx_template to eagerx_ode.
* Update the PACKAGE_NAME variable in Makefile to be eagerx_ode instead of eagerx_template.


Poetry
######

Next we will create a Python package using `Poetry <https://python-poetry.org/>`_.
If you are not familiar with Poetry, we recommend to check out `this article <https://nanthony007.medium.com/stop-using-pip-use-poetry-instead-db7164f4fc72>`_.
It is a very convenient tool for package management.
In the remainder of this section it is assumed that Poetry is installed.

We will populate our empty package as follows (from the root of the repository):

.. code-block:: bash
    :linenos:

    poetry init

Here we specify eagerx and scipy as dependencies, since we will be using scipy to perform the integration of the ODEs.
Furthermore, we specify black, pytest, pytest-cov, flake8 and flake8-bugbear as development dependencies.

Now we are ready to start coding! Note that you can always add or update dependencies later using Poetry.

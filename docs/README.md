## EAGERx Documentation

This folder contains documentation for EAGERx.


### Build the Documentation

#### Install Sphinx and Theme
Execute this command in the project root:
```
pip install -e .[docs]
```

#### Building the Docs

In the `docs/` folder:
```
make html
```

if you want to building each time a file is changed:

```
sphinx-autobuild . _build/html
```

#### Notice

This documentation was adapted from the Stable-baselines3 documentation.

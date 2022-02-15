SHELL=/bin/bash
LINT_PATHS=eagerx/

pytest:
	bash ./scripts/run_tests.sh

check-codestyle:
	# Reformat using black
	poetry run black --check -l 127 ${LINT_PATHS}

lint:
	# stop the build if there are Python syntax errors or undefined names
	# see https://lintlyci.github.io/Flake8Rules/
	poetry run flake8 ${LINT_PATHS} --count --select=E9,F63,F7,F82 --show-source --statistics
	# exit-zero treats all errors as warnings.
	poetry run flake8 ${LINT_PATHS} --count --exit-zero --statistics

.PHONY: check-codestyle
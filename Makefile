# Adapted from DLR-RM/stable-baselines3

SHELL=/bin/bash
LINT_PATHS=eagerx/

pytest:
	bash ./scripts/run_tests.sh

check-codestyle:
	# Reformat using black
	poetry run black --check -l 127 ${LINT_PATHS}

codestyle:
	# Reformat using black
	poetry run black -l 127 ${LINT_PATHS}

lint:
	# stop the build if there are Python syntax errors or undefined names
	# see https://lintlyci.github.io/Flake8Rules/
	poetry run flake8 ${LINT_PATHS} --count --select=E9,F63,F7,F82 --show-source --statistics
	# exit-zero treats all errors as warnings.
	poetry run flake8 ${LINT_PATHS} --count --exit-zero --statistics

# Build docker images
# If you do export RELEASE=True, it will also push them
docker:
	make docker-cpu docker-gpu docker-sb-cpu docker-sb-gpu

docker-cpu:
	./scripts/build_docker.sh

docker-gpu:
	USE_GPU=True ./scripts/build_docker.sh

docker-sb-cpu:
	ADD_SB=True ./scripts/build_docker.sh

docker-sb-gpu:
	ADD_SB=True USE_GPU=True ./scripts/build_docker.sh

# Make docs
doc:
	cd docs && make html

spelling:
	cd docs && make spelling

clean:
	cd docs && make clean

.PHONY: check-codestyle

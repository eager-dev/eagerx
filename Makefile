SHELL=/bin/bash
LINT_PATHS=eagerx/

pytest:
	bash ./scripts/run_tests.sh

check-codestyle:
	# Reformat using black
	black --check -l 127 ${LINT_PATHS}

.PHONY: check-codestyle
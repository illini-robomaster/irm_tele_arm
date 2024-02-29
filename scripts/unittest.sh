#!/bin/bash
find . \( -not -path "./venv/*" -not -path "./thirdparty/*" \) -type f -not -executable -name "*.py" -exec /bin/bash -c 'echo "========== {} =========="; python3 {}' \;

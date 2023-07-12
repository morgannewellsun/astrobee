
### Workflow for installing debians on the robots

1. Ensure that `./requirements/debian_requirements.txt` contains a complete list of end-goal debian packages to install. 
2. Run `./scripts/debian_install` on the local system.

### Workflow for uninstalling debians on the robots

1. On Astrobee, run `./scripts/debian_uninstall` to uninstall debians.
2. Remove the tool folder on Astrobee to complete the uninstall.

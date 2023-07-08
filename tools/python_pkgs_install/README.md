
### Workflow for installing debians on the robots

1. Ensure that `requirements/debian_requirements_root.txt` contains a complete list of end-goal debian packages to install. 
2. On the online machine, run `scripts/collect_debian_requirements` to obtain a recursive list of requirements.
3. Transfer this entire tool folder to Astrobee. 
4. On Astrobee, run `scripts/filter_debian_requirements` to identify which debians need to be installed.
5. Transfer the generated `requirements/debian_requirements_filtered.txt` back to the online machine.
6. On the online machine, run `scripts/download_debians` to download all debian requirements.
7. Transfer the downloaded `debian_downloads/*` to Astrobee.
8. On Astrobee, run `scripts/install_debians` to install debians.

### Workflow for installing python wheels on the robots

1. Ensure that `requirements/wheel_requirements_root.txt` contains a complete list of end-goal pip packages to install.
2. On the online machine, run `scripts/download_wheels` to download wheels.
3. Transfer the downloaded `wheel_downloads` to Astrobee.
4. On Astrobee, run `scripts/install_wheels` to install wheels and record changes in the Python environment.

### Workflow for uninstalling python wheels on the robots

1. On Astrobee, run `scripts/uninstall_wheels` to uninstall wheels.
2. If problems with Python dependency structures arise, consider downgrading to the packages listed in `requirements/downgrade.txt`.

### Workflow for uninstalling debians on the robots

1. On Astrobee, run `scripts/uninstall_debians` to uninstall debians.

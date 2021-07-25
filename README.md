![McGill Rocket Team Logo](https://raw.githubusercontent.com/McGillRocketTeam/ground-station-2019/master/media/MRT-logo.png)
# avionics-2022
Contains all projects in Avionics other than ground-station. 

## Branching rules

* The ___master___ branch will be downloaded for competition. This means that it will contain all final changes.
* The ___dev___ branch will be the working copy before all final versions get merged into "master". 
* All ___feature___ branches should branch off ___dev___. 

## Feature branch naming convention

* yourname/featureName

Eg. Branching off ___dev___ to create a feature branch
```
git checkout dev
git checkout -b mei/anechoicChamberTest
```

## Folder directory rules

Create folders for each project and put your project-specific files in the respective folders to avoid merge conflicts. Feel free to create any necessary subfolders to make everything cleaner.

Eg. 

```

Avionics
.
├── Ejection
|   └── Testing
├── Flight Computer
|   └── Ejection
|   └── Telemetry
|   └── Video recorder
|   └── Sensors
|   └── RTOS
|   └── Release
├── Radios
|   └── Testing
└── Antenna
|   └── Testing
└── Radar
    └── Testing
            
```

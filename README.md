# BioTracker Basetracker

This repository is the basis for new tracking algorithms for the BioTracker.
Please note that tracking algorithms are only certain to work with BioTracker if they are 
compiled with the same compiler using the same C++ standard library.

## How to create your own tracking algorithm

* Clone this Basetracker repository to your local machine 
```bash
git clone git@github.com:BioroboticsLab/biotracker_basetracker.git <your_new_tracker_name>
```
* Create new github repository for _\<your_new_tracker_name\>_
* Change remote of your cloned repository to your newly created one
```bash
cd <your_new_tracker_name>
git remote set -url git@github.com:<your_github_user>/<your_new_tracker_name>.git
git remote add base git@github.com:BioroboticsLab/biotracker_basetracker.git
git push -u origin
```

If the Tracker interface changes you can still pull the changes from the Basetracker to adapt to future 
versions of the interface. This can be done as follows:
```bash
git pull base master
```

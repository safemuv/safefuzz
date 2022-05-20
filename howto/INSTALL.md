# SAFEMUV/ROS installation process

Requirements: Ubuntu 18.04

## First installation phase

This installs ROS and related dependencies, and the SAFEMUV MRS simulator.
At a terminal relative to this repository please run:

```
./howto/scripts/install.sh
```

During the install process:
Whenever asked, enter "Y" to "Do you want to continue" and press Enter

When "GRVC UAL... Select the backends that you want to use:"
please select only "MAVROS", enter y for MAVROS and press Enter
Enter N for the others
Would you like to install needed dependencies, enter "Y"

# Second phase
Following the completion of the previous script,
download the AFI image file and place it under ~/afi_images/afi_core.tar

The AFI image file is present [here](https://dropit.uni.lu/invitations?share=68bf0077edb41b541037&dl=0)

Then please log out and log in again before continuing 
(this is needed to refresh the permissions so Docker can run)

The second phase installs the scripts for the repository:

```
./howto/scripts/install2.sh
```

## Middleware installation
This phase installs the middleware components

```
source ./howto/scripts/middleware-install.sh
```

## Eclipse installation
Please proceed with the [Eclipse installation instructions](./eclipse-instructions.md)

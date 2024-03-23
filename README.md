# Trajectory-Planning-and-Control-for-Omniwheel-Based-Moblie-5DoF-Manipulator

# README File

## Purpose of the Repository

This repository contains a collection of functions designed to perform various tasks related to a pick and place operation. All functions are located in the same folder, and there is an additional folder named 'mr' which should also be in the same directory.

## Main Script - main.m

The `main.m` script serves as the primary entry point for executing the pick and place task. It calls upon various functions to handle different scenarios and cases, including:

### Cases Handled by `main.m`

1. **Best Case**: This case represents the optimal scenario.
2. **Overshoot Case**: This case simulates situations where the system overshoots the desired position.
3. **Low Max Velocity Case**: This case tests scenarios with a low maximum velocity setting.
4. **New Task Case**: This case represents a new task or scenario.

For each of these cases, the `main.m` script exports a corresponding CSV file with the following naming conventions:

- `BestCase.csv`
- `OvershootCase.csv`
- `LowMaxVelocity.csv`
- `NewTask.csv`

## Test Cases

In addition to executing the pick and place task, the `main.m` script also runs test cases for individual functions to ensure their proper functionality.

### Test Cases Included in `main.m`

1. **Trajectory Generation Function**: Tests the trajectory generation function and exports `EETrajectory.csv`.
2. **Next State Function**: Tests the `nextState` function and exports `NextStateTest.csv`.
3. **Feedback Control Function**: Tests the feedback control function.

## Folder Structure

- **Functions Folder**: Contains all the individual functions.
- **mr Folder**: Additional folder required for the repository.

## How to Use

1. Ensure all functions and the `mr` folder are located in the same directory as `main.m`.
2. Run the `main.m` script to execute the pick and place task, along with the test cases.
3. Review the exported CSV files to analyze the results for each case and test.

## Notes

Please make sure to maintain the folder structure and file naming conventions as specified to ensure smooth execution and compatibility.

For any questions or issues, feel free to contact lambamanvir@gmail.com.

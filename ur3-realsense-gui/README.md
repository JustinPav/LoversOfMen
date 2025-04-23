# ur3-realsense-gui/ur3-realsense-gui/README.md

# UR3 and RealSense GUI Project

This project provides a graphical user interface (GUI) for controlling a UR3 robot and a RealSense camera. The application allows users to initiate various processes related to both devices through a user-friendly interface.

## Project Structure

```
ur3-realsense-gui
├── src
│   ├── main.py               # Entry point of the application
│   ├── gui                   # Contains GUI related files
│   │   ├── __init__.py
│   │   ├── main_window.py     # Main window layout and buttons
│   │   └── styles.py         # Styling information for GUI components
│   ├── controllers           # Contains controllers for device operations
│   │   ├── __init__.py
│   │   ├── ur3_controller.py  # Manages UR3 robot operations
│   │   └── realsense_controller.py # Manages RealSense camera operations
│   ├── services              # Contains services for device interactions
│   │   ├── __init__.py
│   │   ├── ur3_service.py     # Provides functionality for UR3 robot
│   │   └── realsense_service.py # Provides functionality for RealSense camera
│   └── utils                 # Contains utility functions
│       ├── __init__.py
│       └── helpers.py        # Utility functions for various tasks
├── requirements.txt          # Lists project dependencies
└── .gitignore                # Specifies files to ignore in version control
```

## Setup Instructions

1. Clone the repository:
   ```
   git clone <repository-url>
   cd ur3-realsense-gui
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Run the application:
   ```
   python3 src/main.py
   ```

## Usage Guidelines

- The main window provides buttons to initiate processes for both the UR3 robot and the RealSense camera.
- Follow the on-screen instructions to operate the devices.

## Components Description

- **GUI**: The graphical interface for user interaction.
- **Controllers**: Manage the operations of the UR3 robot and RealSense camera.
- **Services**: Provide the underlying functionality for device communication.
- **Utilities**: Helper functions for logging and data processing.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.
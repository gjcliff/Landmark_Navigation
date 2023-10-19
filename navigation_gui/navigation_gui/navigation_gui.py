import sys
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget,
                             QScrollArea, QMessageBox, QHBoxLayout, QSizePolicy, QLabel)
from PyQt5 import QtGui
from PyQt5.QtCore import Qt, QTimer
from std_srvs.srv import SetBool
from landmark_manager.srv import SaveLandmark
from landmark_manager.srv import NavigateToLandmark

class NavigationGUI(QMainWindow):
    """
    A GUI application for navigation control, allowing users to save landmarks, cancel navigation,
    and navigate to landmarks
    """

    def __init__(self, node):
        """
        Initializes the NavigationGUI with necessary UI elements and ROS node
        """
        super().__init__()

        # Store the ROS node and set up basic GUI properties
        self.node = node
        self.setWindowTitle('Navigation GUI')
        self.setGeometry(100, 100, 600, 950)

        # Load landmarks
        landmarks_path = self.get_landmarks_file_path()
        self.landmarks = self.load_landmarks(landmarks_path)

        # Set up main layout
        layout = QVBoxLayout()

        # Button for saving landmark
        self.save_button = QPushButton('Save Landmark', self)
        self.save_button.setFont(QtGui.QFont('Arial', 18))
        self.save_button.setMinimumSize(400, 200)
        self.save_button.clicked.connect(self.save_landmark)
        layout.addWidget(self.save_button)

        # Button for canceling navigation
        self.cancel_button = QPushButton('Cancel Navigation', self)
        self.cancel_button.setFont(QtGui.QFont('Arial', 18))
        self.cancel_button.setMinimumSize(400, 200)
        self.cancel_button.clicked.connect(self.cancel_navigation)
        layout.addWidget(self.cancel_button)

        # # Button for navigating to a landmark
        # self.navigate_button = QPushButton('Navigate to Landmark', self)
        # self.navigate_button.setFont(QtGui.QFont('Arial', 14))
        # self.navigate_button.setMinimumSize(400, 100)
        # self.navigate_button.clicked.connect(self.navigate_to_landmark)
        # layout.addWidget(self.navigate_button)

        # Add spacing between the button and instruction label
        layout.addSpacing(20)

        # Add the instruction label
        instruction_label = QLabel("Select a saved landmark:")
        instruction_label.setFont(QtGui.QFont('Arial', 16))
        instruction_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(instruction_label)

        # Keep track of the selected button
        self.selected_button = None

        # Display landmarks in a scroll area with buttons
        self.selected_landmark = None
        scroll_area = QScrollArea(self)

        # Apply custom style to the scroll bar
        scroll_area.setStyleSheet("QScrollBar:vertical { width: 40px; background: lightgrey; }")

        landmarks_widget = QWidget()
        landmarks_layout = QVBoxLayout()
        landmarks_layout.setAlignment(Qt.AlignCenter)

        # Dynamically create buttons for each landmark
        for landmark, coords in self.landmarks.items():
            button = QPushButton(f"{landmark}: x={coords['x']}, y={coords['y']}", self)
            button.setFont(QtGui.QFont('Arial', 14))
            button.setMinimumSize(400, 120)
            button.setMaximumSize(400, 120)
            button.clicked.connect(lambda state, button=button, landmark=landmark:
                                self.select_landmark(button, landmark))

            h_layout = QHBoxLayout()
            h_layout.addStretch(1) # Add stretch to left
            h_layout.addWidget(button) # Add button to the horizontal layout
            h_layout.addStretch(1) # Add stretch to right

            landmarks_layout.addLayout(h_layout)

        landmarks_widget.setLayout(landmarks_layout)
        landmarks_widget.setSizePolicy(QSizePolicy.Preferred,
                                       QSizePolicy.Fixed) # Adjust widget size to its content
        scroll_area.setWidgetResizable(True) # Let the scroll area adjust to content size
        scroll_area.setWidget(landmarks_widget)
        layout.addWidget(scroll_area)

        main_widget = QWidget()
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

    def save_landmark(self):
        """
        Saves a new landmark by calling a ROS service with a landmark. Displays success or error
        messages based on the service call outcome.

        Args: None

        Returns None
        """
        # Create a request to send to the service
        request = SaveLandmark.Request()
        request.name = "Sample Landmark" # Placeholder for now
        request.x = 0.0 # Placeholder for now
        request.y = 0.0 # Placeholder for now

        # Create a client to call the service
        client = self.node.create_client(SaveLandmark, "save_landmark")

        if not client.wait_for_service(timeout_sec=3.0):
            self.show_error('Save landmark service not available')
            return

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the result of the service call
        if future.result() is not None:
            self.show_info('Landmark saved successfully')
        else:
            self.show_error('Failed to call save landmark service')

    def cancel_navigation(self):
        """
        Cancels navigation by calling a ROS service. Displays success or error messages based on
        the service call outcome.

        Args: None

        Returns: None
        """
        # Show a confirmation dialog before canceling navigation
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle('Cancel Navigation')
        msg_box.setText('Are you sure you want to cancel navigation?')
        msg_box.setIcon(QMessageBox.Question)

        # Add "Yes" and "No" buttons to the dialog box
        yes_button = msg_box.addButton('Yes', QMessageBox.YesRole)
        no_button = msg_box.addButton('No', QMessageBox.NoRole)

        # Apply custom style to the buttons
        yes_button.setStyleSheet(
            'QPushButton { min-width: 150px; min-height: 70px; font-size: 16px; }')
        no_button.setStyleSheet(
            'QPushButton { min-width: 150px; min-height: 70px; font-size: 16px; }')

        msg_box.setDefaultButton(no_button)

        # Execute the dialog box and wait for user input
        msg_box.exec_()

        # If the "Yes" button is clicked
        if msg_box.clickedButton() == yes_button:
            # Create a request to send to the service
            request = SetBool.Request()

            # Create a client to call the service
            client = self.node.create_client(SetBool, "cancel_navigation")

            if not client.wait_for_service(timeout_sec=3.0):
                self.show_error('Cancel navigation service not available')
                return

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            # Check the result of the service call
            if future.result() is not None:
                if future.result().success:
                    self.show_info(future.result().message)
                else:
                    self.show_error(future.result().message)
            else:
                self.show_error('Failed to call cancel navigation service')
        # If the "No" button is clicked
        elif msg_box.clickedButton() == no_button:
            pass

    def load_landmarks(self, filepath):
        """
        Loads landmarks from a YAML file

        Args: filepath: Path to the YAML file containing landmarks

        Returns: Dictionary containing loaded landmarks
        """
        with open(filepath, 'r') as file:
            return yaml.safe_load(file)

    def get_landmarks_file_path(self):
        """
        Gets the file path of landmarks.yaml in the package directory

        Args: None

        Returns: Path to landmarks.yaml
        """
        package_path = get_package_share_directory('landmark_manager')
        return os.path.join(package_path, 'config', 'landmarks.yaml')

    def select_landmark(self, button, landmark):
        """
        Handles the landmark selection, updating the UI and internal state

        Args: button: QPushButton associated with the landmark
              landmark: Name of the selected landmark

        Returns: None
        """
        # If a button was previously selected, revert its color
        if self.selected_button:
            self.selected_button.setStyleSheet("")

        # Change the color of the selected button
        button.setStyleSheet("background-color: yellow")  

        # Update the selected button reference
        self.selected_button = button

        self.selected_landmark = landmark
        self.show_info(f"Selected landmark: {landmark}")

    def navigate_to_landmark(self):
        """
        Initiates navigation to a selected landmark by calling a ROS service. Displays warnings or
        navigational status based on the service call outcome.

        Args: None

        Returns: None
        """
        # Check if there is any landmark selected for navigation
        if not self.selected_landmark:
            self.show_warning("No landmark selected for navigation")
            return

        # Retrieve the coordinates of the selected landmark
        landmark_coords = self.landmarks.get(self.selected_landmark)
        if not landmark_coords:
            self.show_warning(f"No coordinates found for landmark \
                                        {self.selected_landmark}")
            return

        # Create a client to call the service
        client = self.node.create_client(NavigateToLandmark, "navigate_to_landmark")

        if not client.wait_for_service(timeout_sec=3.0):
            self.show_error('Navigate to landmark service not available')
            return

        # Create a request to send to the service
        request = NavigateToLandmark.Request()
        request.name = self.selected_landmark

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the result of the service call
        if future.result() is not None:
            self.show_info(f"Navigating to {self.selected_landmark}")
        else:
            self.show_error('Failed to call navigate to landmark service')

    def show_message(self, message, title, icon):
        """
        Displays a message box with a specified message, title, and icon

        Args: message: Text message to display
              title: Title of the message box
              icon: Icon type for the message box

        Returns: None
        """
        # Create a message box
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle(title)
        msg_box.setText(message)
        msg_box.setIcon(icon)
        msg_box.addButton(QMessageBox.Ok)

        # Get the OK button for customization
        ok_button = msg_box.button(QMessageBox.Ok)
        ok_button.setText('OK')
        ok_button.setStyleSheet(
            'QPushButton { min-width: 150px; min-height: 70px; font-size: 16px; }')

        # Set a timer to automatically close the message box after 2 seconds (2000 milliseconds)
        QTimer.singleShot(2000, msg_box.close)

        # Execute the message box and wait for user input
        msg_box.exec_()

    def show_info(self, message):
        """
        Displays an informational message box

        Args: message: Text message to display

        Returns: None
        """
        self.show_message(message, "Info", QMessageBox.Information)

    def show_error(self, message):
        """
        Displays an error message box

        Args: message: Error message to display

        Returns: None
        """
        self.show_message(message, "Error", QMessageBox.Critical)

    def show_warning(self, message):
        """
        Displays a warning message box

        Args: message: Warning message to display

        Returns: None
        """
        self.show_message(message, "Warning", QMessageBox.Warning)

    def closeEvent(self, event):
        """
        Handles the event when the window is closed, ensuring ROS is properly shut down

        Args: event: Close event

        Returns: None
        """
        rclpy.shutdown()
        event.accept()

def main(args=None):
    """
    The main function
    """
    rclpy.init(args=args)
    node = Node("navigation_gui")
    app = QApplication(sys.argv)
    gui = NavigationGUI(node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
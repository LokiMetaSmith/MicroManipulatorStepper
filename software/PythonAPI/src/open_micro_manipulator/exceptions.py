class OpenMicroManipulatorError(Exception):
    """Base exception for OpenMicroManipulator."""
    pass

class DeviceNotFoundError(OpenMicroManipulatorError):
    """Raised when the device cannot be found or connected to."""
    pass

class DeviceBusyError(OpenMicroManipulatorError):
    """Raised when the device is busy and cannot process the command."""
    pass

class CommandTimeoutError(OpenMicroManipulatorError):
    """Raised when a command times out."""
    pass

class CommandError(OpenMicroManipulatorError):
    """Raised when the device returns an error for a command."""
    pass

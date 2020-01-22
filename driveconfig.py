{
    "rightControllerTypes": ["WPI_TalonFX", "WPI_TalonFX"],
    "leftControllerTypes": ["WPI_TalonFX", "WPI_TalonFX"],
    # Ports for the left-side motors
    "leftMotorPorts": [11, 12],
    # Ports for the right-side motors
    "rightMotorPorts": [13, 14],
    # Inversions for the left-side motors
    "leftMotorsInverted": [True, True],
    # Inversions for the right side motors
    "rightMotorsInverted": [False, False],
    # Wheel diameter (in units of your choice - will dictate units of analysis)
    # UNITS OF ANALYSIS: METERS
    "wheelDiameter": 0.1524,
    # If your robot has only one encoder, remove all of the right encoder fields
    # Encoder pulses-per-revolution (*NOT* cycles per revolution!)
    # This value should be the pulses per revolution *of the wheels*, and so
    # should take into account gearing between the encoder and the wheels
    "encoderPPR": 4096,
    # Whether the left encoder is inverted
    "leftEncoderInverted": False,
    # Whether the right encoder is inverted:
    "rightEncoderInverted": False,
    # Your gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "None",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX)
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "leftSlave" (Pigeon on the left slave Talon SRX/FX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "",
}

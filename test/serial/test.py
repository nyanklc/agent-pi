import serial

# Configure the serial port
serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Replace 'COM1' with your serial port and set the appropriate baud rate

while True:
    # Take input from the user
    message = input("Enter the message to send (or 'quit' to exit): ")
    print (message)

    # Check if the user wants to quit
    if message.lower() == 'quit':
        break

    # Send the message through the serial port
    serial_port.write(message.encode())

# Close the serial port
serial_port.close()

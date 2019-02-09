import time

from serial_communication import SerialCommunication


if __name__ == '__main__':
    global serialComm

    serialComm = SerialCommunication()

    if serialComm.open() == False:
        exit(1)

    time.sleep(1)

    i = 0

    while True:

        start = time.time()

        msg = "&" + str(i) + ";" + str(i+1) + ";" + str(i+2) + ";*\n"
        i+=1

        print("----------------")
        print(msg)
        #serialComm.write_message(msg)

        # infinite loop reading data that comes from the microcontroller
        serialComm.read()   

        p = time.time() - start

        #print(1/p)



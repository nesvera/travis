import time

from serial_communication import SerialCommunication


if __name__ == '__main__':
    global serialComm

    serialComm = SerialCommunication()

    if serialComm.open() == False:
        exit(1)

    time.sleep(2)

    
    # speed and steering test via keyboard
    while True:

        print("Enter speed value and steering between -1 and 1: ")
        speed = input()
        steering = input()

        msg = "&" + str(speed) + ";" + str(steering) + ";" + str(0) + ";*\n"

        print("----------------")
        print(msg)
        serialComm.write_message(msg)

        # infinite loop reading data that comes from the microcontroller
        serialComm.read()   
        #time.sleep(0.1)


    '''
    i = 0
    # speed and steering test
    while True:
        
        while True:

            msg = "&" + str(0) + ";" + str(i) + ";" + str(0) + ";*\n"

            print("----------------")
            print(msg)
            serialComm.write_message(msg)

            # infinite loop reading data that comes from the microcontroller
            serialComm.read()   
            #time.sleep(0.1)

            i += 0.1

            if i > 1:
                break
            
            time.sleep(0.05)

        while True:

            msg = "&" + str(0) + ";" + str(i) + ";" + str(0) + ";*\n"

            print("----------------")
            print(msg)
            serialComm.write_message(msg)

            # infinite loop reading data that comes from the microcontroller
            serialComm.read()   
            #time.sleep(0.1)

            i -= 0.1

            if i < -1:
                break

            time.sleep(0.05)

        while True:

            msg = "&" + str(i) + ";" + str(0) + ";" + str(0) + ";*\n"

            print("----------------")
            print(msg)
            serialComm.write_message(msg)

            # infinite loop reading data that comes from the microcontroller
            serialComm.read()   
            #time.sleep(0.1)

            i += 0.1

            if i > 1:
                break
            
            time.sleep(0.5)

        while True:

            msg = "&" + str(i) + ";" + str(0) + ";" + str(0) + ";*\n"

            print("----------------")
            print(msg)
            serialComm.write_message(msg)

            # infinite loop reading data that comes from the microcontroller
            serialComm.read()   
            #time.sleep(0.1)

            i -= 0.1

            if i < -1:
                break

            time.sleep(0.5)
    '''

    '''
    # echo test
    i = 0

    while True:

        start = time.time()

        msg = "&" + str(i) + ";" + str(i+1) + ";" + str(i+2) + ";*\n"
        i+=0.1

        print("----------------")
        print(msg)
        serialComm.write_message(msg)

        # infinite loop reading data that comes from the microcontroller
        serialComm.read()   
        time.sleep(0.1)

        p = time.time() - start

        #print(1/p)
    '''


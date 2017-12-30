def get_status(comms_handler):
    """
    return Packet - response of get_status message
    """
    mess = Message(MTYPE['get_status'])
    return comms_handler.send_message(mess)

def get_fnr(comms_handler):
    """
    return Packet - response of get_fnr message
    """
    mess = Message(MTYPE['get_fnr'])
    return comms_handler.send_message(mess)

def get_speed(comms_handler):
    """
    return Packet - response of get_speed message
    """
    mess = Message(MTYPE['get_speed'])
    return comms_handler.send_message(mess)

def get_battery(comms_handler):
    """
    return Packet - response of get_battery message
    """
    mess = Message(MTYPE['get_battery'])
    return comms_handler.send_message(mess)

def get_power(comms_handler):
    """
    return Packet - response of get_power message
    """
    mess = Message(MTYPE['get_power'])
    return comms_handler.send_message(mess)

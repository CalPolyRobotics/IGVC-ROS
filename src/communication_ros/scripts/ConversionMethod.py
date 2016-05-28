# Will conver a str to int, if empty string, returns 0
def byte_int(s):
   s = s.strip()
   if len(s) == 0:
      return 0
   else:
      byte = s.encode("hex")
      return int(byte, 16)

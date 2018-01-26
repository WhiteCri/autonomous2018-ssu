static int m = 0;

class PlatformTXdata
{
public:
    uint8_t TXarray[14];
    void setTXdata(int speed, int steer, int brake);

private:
    void setData();
};

void PlatformTXdata::setTXdata(int speed, int steer, int brake)
{
    TXarray[6] =  speed*10;
    TXarray[7] =  0x00;
    TXarray[8] =  (steer * 71)>> 8;
    TXarray[9] =  steer*71;
    TXarray[10] = brake;
    setData();
}

void PlatformTXdata::setData()
{
    TXarray[0] = 0x53; // S : 0x53
    TXarray[1] = 0x54; // T : 0x54
    TXarray[2] = 0x58; // X : 0x58
    TXarray[3] = 0x01; // AorM -> 0x00 : manual , 0x01 : auto
    TXarray[4] = 0x00; // E-STOP -> 0x00 : off , 0x01 : on
    TXarray[5] = 0x00; // GEAR -> 0x00 :foward , 0x01 : neutral , 0x02 : backward
    TXarray[11] = m%0xff;// ALIVE -> increasing each one step
    TXarray[12] = 0x0D;// ETX0
    TXarray[13] = 0x0A;// ETX1

    ++m;
}

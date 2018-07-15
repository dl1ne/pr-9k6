#ifndef MENU_h
#define MENU_h

class MENU {
  private:
    float qrgRX_f = 433.000;
    float qrgTX_f = 433.000;
    String input = String("433.000");
  public:
    float qrg();
    void execute();
    float get_rx();
    float get_tx();
};

#endif

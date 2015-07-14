Node used to handle IMU, accelerometers and other devices connected to the ADC or IO cards

--- 

Nodes:
  
  * adc_scope: a oscilloscope like node for the ADC pci9116 card. Useful for plotting with rqt. Can be used for debug
  * io_scope: a node for the pci7432 card, outpus the state (1 or 0) of each line. Can be used for debug

Utitiles:
  python interfaces for both the p9116 and p7432 cards.

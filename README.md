# domeshow-PICL

Firmware for domeshow/lightshow boards (2017) using a [PIC24FJ32GA102](http://ww1.microchip.com/downloads/en/DeviceDoc/39951C.pdf) microcontroller.

## Getting Started

How to get a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

* [MPLAB® X IDE v4.01](http://www.microchip.com/mplab/mplab-x-ide) - See the Downloads section.

### Project Setup

Clone the domeshow-PICL repository
```
$ git clone https://github.com/teslaworksumn/domeshow-PICL.git
```

Start MPLAB. Go to File > Open Project and navigate to the domeshow-PICL directory. Select the DomeShow.X project and click Open Project to open it.

Confirm that the project is configured for the correct device by going to Production > Set Project Configuration > Customize. From here, make sure that the text in the Device field is **PIC24FJ32GA102**

Clean and build the project by going to Production > Clean and Build Project (DomeShow). You can also do this by clicking the icon with a hammer and broom in the main toolbar.

## Authors

* Ryan Fredlund
* Katie Manderfeld
* Ian Smith

## Contributing

When contributing to this repository, please first discuss the change you wish to make via issue, email, or any other method with the authors of this repository before making a change

### Pull Request Process

1. Ensure that your code can clean and build successfully.
2. Assign one or more of the authors to review your Pull Request.
3. You may merge the Pull Request once you have approval from at least one of the authors. If you do not have permission to merge the Pull Request, you may request the reviewer to merge it for you.

## Resources

* [Device datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/39951C.pdf)
* [Detailed UART documentation](http://ww1.microchip.com/downloads/en/DeviceDoc/en026583.pdf)

## Acknowledgements

* Microchip forum user rbreesems for [UART ISR code](http://www.microchip.com/forums/m355304.aspx)
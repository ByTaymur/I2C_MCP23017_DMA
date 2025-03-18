MCP23017 I/O Port Expander with STM32F446 using DMA
This project demonstrates how to interface an MCP23017 16-bit I/O Port Expander with an STM32F446 microcontroller using I2C communication with DMA (Direct Memory Access) for efficient data transfer.
Overview
The MCP23017 is a popular I/O expander that provides 16 additional general-purpose I/O pins using I2C communication. This project shows how to:

Configure and initialize the MCP23017
Use DMA for efficient I2C communication
Test all 16 I/O pins systematically
Perform software reset of the I2C bus for handling lockups

Hardware Requirements

STM32F446 based development board (e.g., NUCLEO-F446RE)
MCP23017 I/O port expander
LEDs or other output devices to visualize the pin states
Pull-up resistors for I2C lines (4.7kΩ recommended)
Connections for UART debugging

Wiring

Connect STM32F446 I2C2 pins to MCP23017:

PB10 (SCL) → SCL of MCP23017
PB11 (SDA) → SDA of MCP23017


Pull-up resistors (4.7kΩ) on both SDA and SCL lines
Connect MCP23017 address pins (A0, A1, A2) to GND for default address (0x20)
Connect MCP23017 GPIO pins to LEDs or other output devices

Software Features

DMA-based I2C Communication: Efficient communication without CPU overhead
Pin Testing Modes:

Sequential testing of individual pins (Port A and Port B)
Cross-pattern testing (Port A and Port B in opposite sequence)


I2C Bus Recovery: Software method to recover from I2C bus lockups
UART Debugging: Outputs test progress and results over UART
Error Handling: Comprehensive error detection and reporting

Development Environment

STM32CubeMX for initial project configuration
Keil MDK or STM32CubeIDE for development
STM32 HAL Libraries

Configuration Details

I2C2 configured for 50kHz with 7-bit addressing
DMA1 Stream2 used for I2C2_RX
DMA1 Stream7 used for I2C2_TX
UART3 configured for 115200 baud rate
LEDs configured for status indication:

LD1 (Green): Port A activity
LD2 (Blue): Port B activity
LD3 (Red): Error indication



Usage

Clone the repository
Open the project in STM32CubeIDE or import into Keil MDK
Build and flash to the STM32F446 device
The program will automatically:

Initialize the MCP23017
Configure all pins as outputs
Run sequential pin tests on all 16 I/O pins
Run cross-pattern tests



Testing Sequence

All pins are turned off
Port A pins (0-7) are tested individually
Port B pins (0-7) are tested individually
Cross-pattern test: Port A ascending (0-7), Port B descending (7-0)
Test sequence repeats continuously

Troubleshooting

If the I2C bus locks up, the software includes a reset mechanism
The red LED (LD3) indicates errors
Check UART output (115200 baud) for detailed error messages
Verify proper pull-up resistors on the I2C lines
Confirm correct I2C address (default is 0x20 with A0=A1=A2=GND)

License
This project is licensed under the MIT License - see the LICENSE file for details.
Acknowledgments

STMicroelectronics for STM32F4 HAL libraries
Microchip for MCP23017 documentation


DMA Kullanarak STM32F446 ile MCP23017 I/O Port Genişletici Uygulaması
Bu proje, MCP23017 16-bit I/O Port Genişletici entegresini STM32F446 mikrodenetleyici ile I2C haberleşmesi üzerinden DMA (Direct Memory Access) kullanarak verimli veri transferi yapmasını göstermektedir.
Genel Bakış
MCP23017, I2C haberleşmesi kullanarak 16 adet ek genel amaçlı I/O pini sağlayan popüler bir port genişleticidir. Bu proje şunları göstermektedir:

MCP23017'nin yapılandırılması ve başlatılması
Verimli I2C haberleşmesi için DMA kullanımı
Tüm 16 I/O pininin sistematik olarak test edilmesi
I2C veri yolu kilitlenmelerini çözmek için yazılımsal sıfırlama yapılması

Donanım Gereksinimleri

STM32F446 tabanlı geliştirme kartı (örn. NUCLEO-F446RE)
MCP23017 I/O port genişletici
Pin durumlarını görselleştirmek için LED'ler veya diğer çıkış cihazları
I2C hatları için pull-up dirençleri (4.7kΩ önerilir)
UART hata ayıklama için bağlantılar

Kablolama

STM32F446 I2C2 pinlerini MCP23017'ye bağlayın:

PB10 (SCL) → MCP23017'nin SCL pini
PB11 (SDA) → MCP23017'nin SDA pini


SDA ve SCL hatlarına pull-up dirençleri (4.7kΩ)
MCP23017 adres pinlerini (A0, A1, A2) varsayılan adres (0x20) için GND'ye bağlayın
MCP23017 GPIO pinlerini LED'lere veya diğer çıkış cihazlarına bağlayın

Yazılım Özellikleri

DMA Tabanlı I2C Haberleşmesi: CPU kullanımı olmadan verimli haberleşme
Pin Test Modları:

Tek tek pin testi (Port A ve Port B)
Çapraz desen testi (Port A ve Port B zıt sıralarda)


I2C Veri Yolu Kurtarma: I2C veri yolu kilitlenmelerinden kurtulmak için yazılımsal yöntem
UART Hata Ayıklama: Test ilerlemesini ve sonuçlarını UART üzerinden gönderir
Hata İşleme: Kapsamlı hata tespiti ve raporlama

Geliştirme Ortamı

Başlangıç proje yapılandırması için STM32CubeMX
Geliştirme için Keil MDK veya STM32CubeIDE
STM32 HAL Kütüphaneleri

Yapılandırma Detayları

I2C2, 50kHz ve 7-bit adresleme için yapılandırılmıştır
I2C2_RX için DMA1 Stream2 kullanılır
I2C2_TX için DMA1 Stream7 kullanılır
UART3, 115200 baud hızı için yapılandırılmıştır
Durum göstergesi için LED'ler yapılandırılmıştır:

LD1 (Yeşil): Port A aktivitesi
LD2 (Mavi): Port B aktivitesi
LD3 (Kırmızı): Hata göstergesi



Kullanım

Repository'yi klonlayın
Projeyi STM32CubeIDE'de açın veya Keil MDK'ya aktarın
Derleyin ve STM32F446 cihazına yükleyin
Program otomatik olarak:

MCP23017'yi başlatacak
Tüm pinleri çıkış olarak yapılandıracak
Tüm 16 I/O pininde sıralı pin testlerini çalıştıracak
Çapraz desen testlerini çalıştıracak



Test Sırası

Tüm pinler kapatılır
Port A pinleri (0-7) tek tek test edilir
Port B pinleri (0-7) tek tek test edilir
Çapraz desen testi: Port A artan (0-7), Port B azalan (7-0)
Test sırası sürekli tekrarlanır

Sorun Giderme

I2C veri yolu kilitlenirse, yazılım bir sıfırlama mekanizması içerir
Kırmızı LED (LD3) hataları gösterir
Ayrıntılı hata mesajları için UART çıkışını (115200 baud) kontrol edin
I2C hatlarında uygun pull-up dirençlerini doğrulayın
Doğru I2C adresini (A0=A1=A2=GND ile varsayılan 0x20) onaylayın

Lisans
Bu proje MIT Lisansı altında lisanslanmıştır - detaylar için LICENSE dosyasına bakın.
Teşekkürler

STM32F4 HAL kütüphaneleri için STMicroelectronics
MCP23017 dokümantasyonu için Microchip

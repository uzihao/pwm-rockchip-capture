# Rockchip RK3568 PWM Capture Driver

## BRIEF

for PWM Capture Mode Standard Usage Flow support in **RK3568** platform in **Linux-4.19.193**, please refer to changes to this file: **driver/pwm/pwm-rockchip-capture.c**

## DTS

You'll need to add something like this in DTS file for your board

```bash
pwm15: pwm@fe700030 {
    compatible = "rockchip,pwm-capture";
	reg = <0x0 0xfe700030 0x0 0x10>;
	interrupts = <GIC_SPI 85 IRQ_TYPE_LEVEL_HIGH>,
			 <GIC_SPI 89 IRQ_TYPE_LEVEL_HIGH>;
	#pwm-cells = <3>;
	pinctrl-names = "default";
	pinctrl-0 = <&pwm15m0_pins>;
	clocks = <&cru CLK_PWM3>, <&cru PCLK_PWM3>;
	clock-names = "pwm", "pclk";
	pwm_channel = <3>;
    status = "okay";
};
```

RK3568 only supports PWM interruption of **PWM 7** / **PWM 11** / **PWM 15**

## Example

```c
struct rk_pwm_capture_data_t {
    u_int64_t period_ns;
    u_int64_t duty_ns;
    u_int8_t wait_time_ms;
};

int main()
{
    int fd = open("/dev/pwm@fe700030@capture", O_RDONLY);
    if (fd < 0) {
        printf("Failed to open device: %d\n\n", errno);
        return -1;
    }

    struct rk_pwm_capture_data_t data;
    memset(&data, '\0', sizeof(data));
    int readBytes = read(fd, &data, sizeof(data));
    printf("ReadBytes: %d; period; %d; duty: %d; wait_time: %d.\n\n", readBytes, data.period_ns, data.duty_ns, data.wait_time_ms);

    close(fd);
    return 0;
}
```


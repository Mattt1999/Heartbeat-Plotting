/*
 * main.cpp
 *
 *  Created on: Oct 31, 2023
 *      Author: matei
 */

#include "max30100.h"

#define SLAVE_ADDRESS 0x57

volatile sig_atomic_t flag = 0;
int fd;

void timer_callback(int signum);

int main(int argc, char **argv) {

	/*Initialize graphical interface*/

	egt::Application app(argc, argv);
	egt::TopWindow win;
	win.color(egt::Palette::ColorId::bg, egt::Color(0x1d2239ff));

	egt::VerticalBoxSizer sizer1;
	win.add(expand(sizer1));

	egt::HorizontalBoxSizer sizer0(egt::Justification::start);
	sizer0.color(egt::Palette::ColorId::bg, egt::Color(0x252c48ff));
	sizer0.color(egt::Palette::ColorId::border, egt::Palette::black);
	sizer0.border(2);
	sizer0.border_flags(egt::Theme::BorderFlag::bottom);
	sizer0.fill_flags(egt::Theme::FillFlag::blend);
	sizer1.add(expand_horizontal(sizer0));

	auto logo = std::make_shared<egt::ImageLabel>(
			egt::Image("icon:egt_logo_white.png;128"));
	logo->margin(10);
	sizer0.add(logo);

	egt::Label label0("Health device");
	label0.color(egt::Palette::ColorId::label_text, egt::Palette::white);
	label0.padding(5);
	label0.font(egt::Font(28));
	sizer0.add(label0);

	egt::HorizontalBoxSizer sizer2;
	sizer1.add(expand(sizer2));

	egt::LineChart line1;
	line1.label("", "BPMs", "Heartbeats");
	line1.font(egt::Font(24));
	line1.line_width(2);
	line1.grid_style(egt::LineChart::GridFlag::box_major_ticks_coord);
	line1.color(egt::Palette::ColorId::label_text, egt::Palette::white);
	line1.color(egt::Palette::ColorId::button_bg, egt::Palette::gray);
	line1.color(egt::Palette::ColorId::button_fg, egt::Color(0xf3c626ff));
	line1.margin(2);
	sizer2.add(expand(line1));

	egt::Label label3("000");
	label3.color(egt::Palette::ColorId::label_text, egt::Color(0xf3c626ff));
	label3.font(egt::Font(80));
	label3.margin(20);
	sizer2.add(center(label3));

	egt::HorizontalBoxSizer sizer3;
	sizer1.add(expand(sizer3));

	egt::LineChart line2;
	line2.label("", "%", "SpO2");
	line2.line_width(2);
	line2.grid_style(egt::LineChart::GridFlag::box_major_ticks_coord);
	line2.color(egt::Palette::ColorId::label_text, egt::Palette::white);
	line2.color(egt::Palette::ColorId::button_bg, egt::Palette::gray);
	line2.color(egt::Palette::ColorId::button_fg, egt::Color(0x51e25fff));
	sizer3.add(expand(line2));

	egt::Label label4("000");
	label4.color(egt::Palette::ColorId::label_text, egt::Color(0x51e25fff));
	label4.font(egt::Font(80));
	label4.margin(20);
	sizer3.add(center(label4));

	egt::FlexBoxSizer sizer4;
	sizer1.add(expand(sizer4));

	egt::Label label6("Temperature");
	label6.color(egt::Palette::ColorId::label_text, egt::Color(0xff1493ff));
	label6.font(egt::Font(50));
	label6.margin(20);
	//label6.align(AlignFlag::center);
	sizer4.add(center(label6));

	egt::Label label5("000");
	label5.color(egt::Palette::ColorId::label_text, egt::Color(0xff1493ff));
	label5.font(egt::Font(80));
	label5.margin(20);
	sizer4.add(right(label5));

	/*Initialize low level sensor communication*/
//	int fd;
	fd = open(DEV_I2C, O_RDWR);
	if (fd < 0) {
		cout << "Error opening the file " << fd << endl;
	} else {
		cout << "file opened ok" << endl;
	}

	//set slave address for the sensor
	if (ioctl(fd, I2C_SLAVE, SLAVE_ADDRESS) < 0) {
		cout << "slave address couldn't be set";
	} else {
		cout << "Slave Addr set" << endl;
	}

	MAX30100 *pulseOx = new MAX30100(DEFAULT_OPERATING_MODE,
	DEFAULT_SAMPLING_RATE, DEFAULT_LED_PULSE_WIDTH,
	DEFAULT_IR_LED_CURRENT, true, true, fd);
	pulseoxymeter_t result;

	volatile float sum = 0;

	egt::PeriodicTimer timer2(std::chrono::milliseconds(10));
	timer2.on_timeout([&]() {
		result = pulseOx->update();
		if (result.pulseDetected == true) {
			cout << "BPM: ";
			cout << result.heartBPM << endl;
			sum += result.heartBPM;
		} else {
			;
		}
	});

	//2.452372752527856026e-1 * x) + (0.50952544949442879485

	timer2.start();

	size_t sample_counter = 0;
	size_t connsecutiveValues = 0;

	int temp_heartBeat = -1;

	egt::PeriodicTimer timer1(std::chrono::milliseconds(1000));
	timer1.on_timeout([&]() {

		static const int chart_limit = 20;

		if (pulseOx->vector_filled == true) {


			if(temp_heartBeat == (int)result.heartBPM)
			{
				connsecutiveValues++;
			}
			else
			{
				connsecutiveValues = 0;
			}

			//if 60 connsecutive values of the pulse are equal we assume there is no pulse
			if(connsecutiveValues >= 40)
			{
				//in this case no pulse is detected. Perhaps replace this code with smth that prints a message
				cout << "NO PULSE "<< endl;
				egt::ChartItemArray data1;
				data1.add(sample_counter, 0);
				line1.add_data(data1);
				while (line1.data_size() > chart_limit)
					line1.remove_data(1);
				label3.text(egt::detail::format(0, 0));

				egt::ChartItemArray data2;
				data2.add(sample_counter, 0);
				line2.add_data(data2);
				while (line2.data_size() > chart_limit)
					line2.remove_data(1);
				label4.text(egt::detail::format(0, 0));

				label5.text(egt::detail::format(0, 0));
			}
			else
			{
				egt::ChartItemArray data1;
				data1.add(sample_counter, result.heartBPM);
				line1.add_data(data1);
				while (line1.data_size() > chart_limit)
					line1.remove_data(1);
				label3.text(egt::detail::format(result.heartBPM, 0));

				egt::ChartItemArray data2;
				data2.add(sample_counter, result.SaO2);
				line2.add_data(data2);
				while (line2.data_size() > chart_limit)
					line2.remove_data(1);
				label4.text(egt::detail::format(result.SaO2, 0));

				label5.text(egt::detail::format(pulseOx->readTemperature(), 0));
			}

			sample_counter++;

			temp_heartBeat = (int)result.heartBPM;

		}
	});
	timer1.start();

	win.show();

	return app.run();
}

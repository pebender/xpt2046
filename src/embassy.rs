use crate::{calibration, driver};
use embedded_hal::{digital::InputPin, spi::SpiDevice};
use embedded_hal_async::digital::Wait;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_futures::select::{select, Either};


trait Driver<Irq>
where 
Irq: InputPin + Wait
{
    type Error;
    type CalibrationData;

    fn init(&mut self);
    fn run_init(&mut self);
    fn run(&mut self, irq: &mut Irq) -> Result<bool, Self::Error>;
    fn set_calibration_data(&mut self, calibration_data: Self::CalibrationData);
     fn get_touch_point(&self) -> driver::Point;
     fn get_touch_point_raw(&self) -> driver::Point;
}

impl<Spi, SpiError, Irq, IrqError> Driver<Irq> for driver::Xpt2046<Spi>
where
    Spi: SpiDevice<u8, Error = SpiError>,
    SpiError: embedded_hal::spi::Error,
        Irq: InputPin<Error = IrqError> + Wait,
        IrqError: embedded_hal::digital::Error,
{
    type Error = driver::Error<SpiError, IrqError>;
    type CalibrationData = driver::CalibrationData;

    fn init(&mut self) {
        self.init().unwrap();
    }
    fn run_init(&mut self) {
        self.run_init();
    }
    fn run(&mut self, irq: &mut Irq) -> Result<bool, Self::Error> {
        self.run(irq)
    }
    fn set_calibration_data(&mut self, calibration_data: Self::CalibrationData) {
        self.set_calibration_data(calibration_data);
    }
    fn get_touch_point(&self) -> Option<driver::Point> {
        match self.is_touched() {
            true => Some(self.get_touch_point()),
            false => None,
        }
    }
    fn get_touch_point_raw(&self) -> Option<driver::Point> {
                match self.is_touched() {
            true => Some(self.get_touch_point_raw()),
            false => None,
        }
    }
}

struct Runner<D, Irq>
where 
D: Driver<Irq>,
Irq: InputPin + Wait
{
    driver: D,
    irq: Irq,

}

impl<D, Irq> Runner<D, Irq>
where
D: Driver<Irq>,
Irq: InputPin + Wait
{
    fn run(&self) {
    {

        self.driver.init();
        self.driver.run_init();
        let mut touch_point = TouchEvent::Up;
        let mut touch_event_raw = TouchEvent::Up;
        loop {
            match select(self.irq.wait_for_low(), self.commands_receiver.receive()).await {
                Either::First(r) => r.map_err(|e| Error::Irq(e))?,
                Either::Second(command) => match command {
                    TouchCommand::UpdateCalibration(calibration_data) => {
                        self.driver.set_calibration_data(&calibration_data);
                    }
                    TouchCommand::UpdateTouchEventMode(mode) => event_mode = mode,
                },
                Either3::Third(_b) => return Ok(()),
            }
            while irq.is_low().map_err(|e| Error::Irq(e))? {
                driver.run(irq)?;
                if driver.is_touched() {
                    let point = driver.get_touch_point();
                    match event_mode {
                        TouchEventMode::Filtered => {
                            if point.x >= 0
                                && point.x < DISPLAY_SIZE.width as i32
                                && point.y >= 0
                                && point.y < DISPLAY_SIZE.height as i32
                            {
                                match event {
                                    TouchEvent::Up => {
                                        event = TouchEvent::Down((point, None));
                                        events.signal(event);
                                    }
                                    TouchEvent::Down((p, _)) | TouchEvent::Move((p, _)) => {
                                        if point != p {
                                            event = TouchEvent::Move((point, None));
                                            events.signal(event);
                                        }
                                    }
                                }
                            }
                        }
                        TouchEventMode::Raw => {
                            let point_raw = driver.get_touch_point_raw();
                            match event {
                                TouchEvent::Up => {
                                    event = TouchEvent::Down((point, Some(point_raw)));
                                    events.signal(event);
                                }
                                TouchEvent::Down(_) | TouchEvent::Move(_) => {
                                    event = TouchEvent::Move((point, Some(point_raw)));
                                    events.signal(event);
                                }
                            }
                        }
                    }
                }
                match select3(
                    Timer::after_micros(500),
                    commands_receiver.receive(),
                    stop.wait(),
                )
                .await
                {
                    Either3::First(_) => {}
                    Either3::Second(command) => match command {
                        TouchCommand::UpdateCalibration(calibration_data) => {
                            driver.set_calibration_data(&calibration_data);
                        }
                        TouchCommand::UpdateTouchEventMode(mode) => event_mode = mode,
                    },
                    Either3::Third(_b) => return Ok(()),
                }
            }
            driver.clear_touch();
            event = TouchEvent::Up;
            events.signal(event);
        }
    }
}
}

enum TouchEvent {
    Down(driver::Point),
    Move(driver::Point),
    Up
}

struct Interface {
    touches: Channel<CriticalSectionRawMutex, (TouchEvent, TouchEvent), 16>
}

impl Interface {
    async fn get_touch_point(&self) -> Touch {
        let (touch_point, touch_point_raw) = self.touches.receive().await;
        touch_point
    }

    async fn get_touch_point_raw(&self) -> Touch {
        let (touch_point, touch_point_raw) = self.touches.receive().await;
        touch_point_raw
    }
}


struct Interface<D> {
    fn_set_calibration_data
}

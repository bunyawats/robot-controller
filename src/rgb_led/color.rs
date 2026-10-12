//! Colours for the RGB LED.

/// A colour as (red, green, blue), each 0-255.
pub type Rgb = (u8, u8, u8);

pub const OFF: Rgb = (0, 0, 0);

/// Colour at `hue` degrees (0 red, 120 green, 240 blue) on the colour wheel, fully
/// saturated, with `max` as the brightest channel.
pub fn rainbow(hue: u32, max: u8) -> Rgb {
    let hue = hue % 360;
    // Within each 60-degree sector one channel ramps up or down while another sits at max.
    let ramp = |deg: u32| (u32::from(max) * deg / 60) as u8;
    let up = ramp(hue % 60);
    let down = max - up;
    match hue / 60 {
        0 => (max, up, 0),
        1 => (down, max, 0),
        2 => (0, max, up),
        3 => (0, down, max),
        4 => (up, 0, max),
        _ => (max, 0, down),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rainbow_hits_the_primaries() {
        assert_eq!(rainbow(0, 32), (32, 0, 0));
        assert_eq!(rainbow(120, 32), (0, 32, 0));
        assert_eq!(rainbow(240, 32), (0, 0, 32));
        assert_eq!(rainbow(360, 32), rainbow(0, 32));
    }

    #[test]
    fn rainbow_never_exceeds_max_or_goes_dark() {
        for hue in 0..360 {
            let (r, g, b) = rainbow(hue, 32);
            assert!(r.max(g).max(b) == 32, "hue {hue}: {:?}", (r, g, b));
        }
    }
}

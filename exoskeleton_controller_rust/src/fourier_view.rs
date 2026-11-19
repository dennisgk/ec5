use crate::protocol::FFT_BINS;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct IgnoreSegment {
    pub enabled: bool,
    pub low_hz: f32,
    pub high_hz: f32,
}

impl Default for IgnoreSegment {
    fn default() -> Self {
        Self {
            enabled: true,
            low_hz: 50.0,
            high_hz: 70.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FourierIgnore {
    pub enabled: bool,
    pub segments: Vec<IgnoreSegment>,
    #[serde(skip)]
    pub mask: [u8; 17],
}

impl Default for FourierIgnore {
    fn default() -> Self {
        Self {
            enabled: false,
            segments: vec![IgnoreSegment::default()],
            mask: [0; 17],
        }
    }
}

pub struct SpectrumPlot {
    pub segments: Vec<Vec<[f64; 2]>>,
    pub y_max: f64,
}

impl FourierIgnore {
    pub fn validate(&self) -> Result<(), String> {
        if self.segments.len() > 16 {
            return Err("At most 16 ignore segments".into());
        }
        for row in &self.segments {
            if !row.low_hz.is_finite()
                || !row.high_hz.is_finite()
                || row.low_hz < 0.0
                || row.high_hz > 500.0
                || row.low_hz > row.high_hz
            {
                return Err("Ignore segment: require 0 <= low <= high <= 500 Hz".into());
            }
        }
        Ok(())
    }

    // The device supplies this mask after Apply. Never derive display exclusions
    // from unapplied editor values or a different floating-point comparison.
    pub fn plot_data(&self, power: &[f32], sample_rate: u16) -> SpectrumPlot {
        let mut segments = Vec::new();
        let mut segment = Vec::new();
        let mut maximum = 0.0_f64;
        for (bin, power) in power.iter().take(FFT_BINS).enumerate() {
            let hz = bin as f64 * sample_rate as f64 / 256.0;
            if !power.is_finite() || self.mask[bin / 8] & (1 << (bin % 8)) != 0 {
                if !segment.is_empty() {
                    segments.push(std::mem::take(&mut segment));
                }
                continue;
            }
            maximum = maximum.max(*power as f64);
            segment.push([hz, *power as f64]);
        }
        if !segment.is_empty() {
            segments.push(segment);
        }
        SpectrumPlot {
            segments,
            y_max: (maximum * 1.05).max(1e-12),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn acknowledged_mask_excludes_peak_and_breaks_plot_line() {
        let mut ignore = FourierIgnore::default();
        ignore.mask[2] = 1; // bin 16, 62.5 Hz
        let mut power = vec![2.0; FFT_BINS];
        power[16] = 1e6;
        let shown = ignore.plot_data(&power, 1000);
        assert_eq!(shown.y_max, 2.1);
        assert_eq!(shown.segments.len(), 2);
        assert_eq!(shown.segments.iter().map(Vec::len).sum::<usize>(), 128);
    }
    #[test]
    fn invalid_and_excessive_ranges_are_rejected() {
        let mut ignore = FourierIgnore::default();
        ignore.segments[0].low_hz = f32::NAN;
        assert!(ignore.validate().is_err());
        ignore.segments = vec![IgnoreSegment::default(); 17];
        assert!(ignore.validate().is_err());
    }
}

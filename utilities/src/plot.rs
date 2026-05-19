//! Interactive line-plot utility built on `egui`.
//!
//! This module provides a small native plotting helper for manual inspection of
//! numeric data. It opens an `eframe` window and renders a single line series
//! from points represented as `[x, y]`.
//!
//! The implementation is intended for interactive use from binaries or example
//! programs rather than automated tests. Typical usage is to prepare a vector
//! of points and pass it to [`plot`].
//!
//! # Examples
//!
//! ```no_run
//! use utilities::plot;
//!
//! let data: Vec<[f64; 2]> = (0..100)
//!     .map(|i| {
//!         let x = i as f64 * 0.1;
//!         [x, x.sin()]
//!     })
//!     .collect();
//!
//! plot(data);
//! ```

use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};

/// Render `data` as a line plot in a native window.
pub fn plot(data: Vec<[f64; 2]>) {
    eframe::run_native(
        "plot",
        eframe::NativeOptions::default(),
        Box::new(|_| Ok(Box::new(PlotApp { data }))),
    )
    .expect("Failed to open plot window");
}

struct PlotApp {
    data: Vec<[f64; 2]>,
}

impl eframe::App for PlotApp {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        if ctx.input(|i| i.key_pressed(egui::Key::Escape) || !i.keys_down.is_empty()) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        egui::CentralPanel::default().show(ctx, |ui| {
            Plot::new("p").show(ui, |pu| {
                pu.line(Line::new(PlotPoints::from_iter(self.data.iter().copied())));
            });
        });
    }
}

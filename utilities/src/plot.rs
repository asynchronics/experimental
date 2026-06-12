//! Interactive line-plot utility built on `egui`.
//!
//! This module provides a small native plotting helper for manual inspection of
//! numeric data. It opens an `eframe` window and renders one or more line
//! series in a native window.
//!
//! Use [`plot`] for a single series of `[x, y]` points, [`plot_indexed`] for a
//! single series of sampled values plotted against their indices, or
//! [`plot_series`] for multiple named series on the same plot.
//!
//! # Examples
//!
//! ```no_run
//! use utilities::plot;
//!
//! let mut data = Vec::with_capacity(100);
//! for i in 0..100 {
//!     let x = i as f64 * 0.1;
//!     data.push([x, x.sin()]);
//! }
//!
//! plot(&data);
//! ```
//!
//! ```no_run
//! use utilities::plot::plot_indexed;
//!
//! let mut data = Vec::with_capacity(100);
//! for i in 0..100 {
//!     let x = i as f64 * 0.1;
//!     data.push(x.sin());
//! }
//!
//! plot_indexed(&data);
//! ```
//!
//! ```no_run
//! use utilities::plot::plot_series;
//!
//! let mut sin = Vec::with_capacity(100);
//! let mut cos = Vec::with_capacity(100);
//! for i in 0..100 {
//!     let x = i as f64 * 0.1;
//!     sin.push([x, x.sin()]);
//!     cos.push([x, x.cos()]);
//! }
//!
//! plot_series(&[("sin", &sin), ("cos", &cos)]);
//! ```

use eframe::egui;
use egui_plot::{Legend, Line, Plot, PlotPoints};

/// Render a single series in a native window, where each point is `[x, y]`.
pub fn plot(data: &[[f64; 2]]) {
    let data = vec![Series {
        name: String::new(),
        points: data.to_vec(),
    }];
    eframe::run_native(
        "plot",
        eframe::NativeOptions::default(),
        Box::new(|_| Ok(Box::new(PlotApp { data }))),
    )
    .expect("Failed to open plot window");
}

/// Render a single series of sampled values using the sample index as `x`.
pub fn plot_indexed(data: &[f64]) {
    let mut points = Vec::with_capacity(data.len());
    for (i, &y) in data.iter().enumerate() {
        points.push([i as f64, y]);
    }
    plot(&points);
}

/// Render multiple named `[x, y]` series on the same plot.
pub fn plot_series(series: &[(&str, &[[f64; 2]])]) {
    let mut data = Vec::with_capacity(series.len());
    for (name, points) in series {
        data.push(Series {
            name: (*name).to_string(),
            points: points.to_vec(),
        });
    }
    eframe::run_native(
        "plot",
        eframe::NativeOptions::default(),
        Box::new(|_| Ok(Box::new(PlotApp { data }))),
    )
    .expect("Failed to open plot window");
}

struct Series {
    name: String,
    points: Vec<[f64; 2]>,
}

struct PlotApp {
    data: Vec<Series>,
}

impl eframe::App for PlotApp {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        if ctx.input(|i| i.key_pressed(egui::Key::Escape) || !i.keys_down.is_empty()) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        egui::CentralPanel::default().show(ctx, |ui| {
            let mut plot = Plot::new("p");
            if self.data.len() > 1 {
                plot = plot.legend(Legend::default());
            }
            plot.show(ui, |pu| {
                for series in &self.data {
                    let mut line = Line::new(PlotPoints::from_iter(series.points.iter().copied()));
                    if !series.name.is_empty() {
                        line = line.name(&series.name);
                    }
                    pu.line(line);
                }
            });
        });
    }
}

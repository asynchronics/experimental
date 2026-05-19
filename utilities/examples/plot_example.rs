use utilities::plot::plot_series;

fn main() {
    let mut sin = Vec::with_capacity(100);
    let mut cos = Vec::with_capacity(100);
    for i in 0..100 {
        let x = i as f64 * 0.1;
        sin.push([x, x.sin()]);
        cos.push([x, x.cos()]);
    }

    plot_series(&[("sin", &sin), ("cos", &cos)]);
}

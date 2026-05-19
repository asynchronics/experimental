use utilities::plot;

fn main() {
    let data: Vec<[f64; 2]> = (0..100)
        .map(|i| {
            let x = i as f64 * 0.1;
            [x, x.sin()]
        })
        .collect();

    plot(data);
}

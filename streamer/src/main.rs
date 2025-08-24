use clap::Parser;
use anyhow::Result;
use std::process::Command;
use std::time::{Duration, Instant};
use tokio::time::sleep;
use rerun;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Video device path (e.g., /dev/video0)
    #[arg(short, long, default_value = "/dev/video0")]
    device: String,

    /// Project name for the stream
    #[arg(short, long)]
    project_name: String,

    /// Label for the stream
    #[arg(short, long)]
    label: String,

    /// Frames per second
    #[arg(short, long, default_value_t = 30)]
    fps: i32,

    #[arg(short, long, default_value = "rerun+http://localhost:5050/proxy")]
    rerun_endpoint: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    
    println!("Starting webcam stream:");
    println!("  Device: {}", args.device);
    println!("  Project: {}", args.project_name);
    println!("  Label: {}", args.label);
    println!("  FPS: {}", args.fps);
    println!("  Rerun endpoint: {}", args.rerun_endpoint);

    let rec = rerun::RecordingStreamBuilder::new(args.project_name.clone())
        .connect_grpc_opts(args.rerun_endpoint.clone(), None)?;

    // Check if ffmpeg is available
    let ffmpeg_check = Command::new("ffmpeg")
        .arg("-version")
        .output();
    
    if ffmpeg_check.is_err() {
        anyhow::bail!("ffmpeg is required but not found. Please install ffmpeg first.");
    }

    // Check if the video device exists
    if !std::path::Path::new(&args.device).exists() {
        anyhow::bail!("Video device {} does not exist", args.device);
    }

    println!("Video device found and ffmpeg available!");
    println!("Starting stream with ffmpeg...");
    println!("Press Ctrl+C to stop");

    let frame_interval = Duration::from_millis((1000 / args.fps) as u64);
    let mut frame_count = 0;
    let start_time = Instant::now();

    loop {
        let frame_start = Instant::now();
        
        // Capture a single frame using ffmpeg to stdout
        let output = Command::new("ffmpeg")
            .args([
                "-f", "v4l2",
                "-i", &args.device,
                "-vframes", "1",
                "-f", "image2",
                "-vf", &format!("drawtext=text='{} - {} (Frame: {})':fontcolor=green:fontsize=24:x=10:y=30", 
                               args.project_name, args.label, frame_count),
                "-c:v", "png", // Use PNG format for better quality
                "pipe:1" // Output to stdout
            ])
            .output();

        match output {
            Ok(result) => {
                if result.status.success() && !result.stdout.is_empty() {
                    // Load image from the captured data
                    if let Ok(img) = image::load_from_memory(&result.stdout) {
                        // Convert image to rerun format and stream
                        rec.log(args.label.clone(), &rerun::Image::from_dynamic_image(img)?)?;
                        
                        frame_count += 1;
                        let elapsed = start_time.elapsed();
                        let fps_actual = frame_count as f64 / elapsed.as_secs_f64();
                        
                        println!("Frame {} streamed to rerun - Actual FPS: {:.2}", frame_count, fps_actual);
                    } else {
                        eprintln!("Failed to decode captured frame");
                    }
                } else {
                    eprintln!("ffmpeg failed: {}", String::from_utf8_lossy(&result.stderr));
                }
            }
            Err(e) => {
                eprintln!("Failed to capture frame: {}", e);
                break;
            }
        }

        // Wait for next frame
        let elapsed = frame_start.elapsed();
        if elapsed < frame_interval {
            sleep(frame_interval - elapsed).await;
        }
    }

    println!("Stream ended. Total frames captured: {}", frame_count);
    
    Ok(())
}

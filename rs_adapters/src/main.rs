use clap::Parser;
use tokio_tungstenite::{connect_async, tungstenite::protocol::Message};
use futures_util::{SinkExt, StreamExt};
use serde_json::Value;
use anyhow::Result;

#[derive(Parser)]
#[command(name = "robot-adapter")]
#[command(about = "Robot adapter that connects to WebSocket API")]
struct Args {
    /// WebSocket URL to connect to
    #[arg(short, long, default_value = "ws://localhost:5000/ws/update_joints")]
    url: String,
}

async fn subscribe_to_websocket(url: &str) -> Result<()> {
    println!("🔌 Connecting to WebSocket: {}", url);
    
    let (ws_stream, _) = connect_async(url).await?;
    println!("✅ Connected to WebSocket successfully!");
    
    let (mut write, mut read) = ws_stream.split();
    
    // Send initial empty message to get current joint state
    let initial_message = serde_json::json!({});
    write.send(Message::Text(initial_message.to_string())).await?;
    println!("📤 Sent initial request for joint state");
    
    // Listen for messages
    while let Some(message) = read.next().await {
        match message? {
            Message::Text(text) => {
                println!("📥 Received: {}", text);
                
                // Parse the JSON response
                if let Ok(json) = serde_json::from_str::<Value>(&text) {
                    if let Some(joints) = json.as_object() {
                        println!("🤖 Joint states:");
                        for (joint_name, joint_data) in joints {
                            if let Some(joint_obj) = joint_data.as_object() {
                                if let (Some(value), Some(limits)) = (
                                    joint_obj.get("value").and_then(|v| v.as_f64()),
                                    joint_obj.get("limits").and_then(|l| l.as_array())
                                ) {
                                    println!("  {}: value={:.3}, limits=[{:.3}, {:.3}]", 
                                        joint_name, value, 
                                        limits[0].as_f64().unwrap_or(0.0),
                                        limits[1].as_f64().unwrap_or(0.0));
                                }
                            }
                        }
                    }
                }
            }
            Message::Close(_) => {
                println!("🔌 WebSocket connection closed");
                break;
            }
            _ => {}
        }
    }
    
    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    
    println!("🤖 Robot Adapter starting...");
    println!("🎯 Target WebSocket: {}", args.url);
    
    subscribe_to_websocket(&args.url).await?;
    
    Ok(())
}

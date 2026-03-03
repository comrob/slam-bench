#!/usr/bin/env python3
"""
Extract reference trajectory from MCAP bagfile containing GPS (NavSatFix) messages.

This tool reads MCAP files offline, extracts GPS data, converts coordinates to
local Cartesian frame using UTM projection, and outputs TUM format trajectory.

The first GPS point is used as the local origin (0, 0, 0).
"""

import argparse
import sys
from pathlib import Path
from typing import List, Tuple, Optional

try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
except ImportError:
    print("Error: mcap library not installed. Install with: pip install mcap mcap-ros2-support", file=sys.stderr)
    sys.exit(1)

try:
    import pyproj
except ImportError:
    print("Error: pyproj library not installed. Install with: pip install pyproj", file=sys.stderr)
    sys.exit(1)


class GPSTrajectoryExtractor:
    """Extract and convert GPS trajectory to TUM format."""
    
    def __init__(self, use_first_as_origin: bool = True):
        """
        Initialize the GPS trajectory extractor.
        
        Args:
            use_first_as_origin: If True, set first GPS point as origin (0,0,0)
        """
        self.use_first_as_origin = use_first_as_origin
        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None
        self.origin_alt: Optional[float] = None
        self.transformer: Optional[pyproj.Transformer] = None
        self.utm_zone: Optional[int] = None
        self.utm_hemisphere: Optional[str] = None
        
    def _initialize_origin(self, lat: float, lon: float, alt: float):
        """Initialize the UTM projection based on the first GPS point."""
        # Determine UTM zone from longitude
        self.utm_zone = int((lon + 180) / 6) + 1
        self.utm_hemisphere = 'north' if lat >= 0 else 'south'
        
        # Store origin
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_alt = alt
        
        # Create UTM projection transformer
        # EPSG:4326 is WGS84 (lat/lon)
        # We create a custom UTM projection centered at our origin
        utm_proj = f"+proj=utm +zone={self.utm_zone} +{'north' if lat >= 0 else 'south'} +datum=WGS84 +units=m +no_defs"
        
        self.transformer = pyproj.Transformer.from_crs(
            "EPSG:4326",  # WGS84 lat/lon
            utm_proj,
            always_xy=True  # Use (lon, lat) order instead of (lat, lon)
        )
        
        print(f"Initialized UTM projection: Zone {self.utm_zone}{self.utm_hemisphere[0].upper()}")
        print(f"Origin: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.3f}")
        
    def _gps_to_local(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """
        Convert GPS coordinates to local Cartesian frame.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
            
        Returns:
            Tuple of (x, y, z) in meters relative to origin
        """
        if self.transformer is None:
            raise RuntimeError("Origin not initialized. Call _initialize_origin first.")
        
        # Transform to UTM coordinates
        x_utm, y_utm = self.transformer.transform(lon, lat)
        
        # If this is the origin point, we should get the origin UTM coords
        if self.use_first_as_origin and self.origin_lat is not None:
            origin_x_utm, origin_y_utm = self.transformer.transform(self.origin_lon, self.origin_lat)
            
            # Subtract origin to get local coordinates
            x = x_utm - origin_x_utm
            y = y_utm - origin_y_utm
            z = alt - self.origin_alt
        else:
            x = x_utm
            y = y_utm
            z = alt
            
        return x, y, z
    
    def extract_from_mcap(self, bagfile: str, topic: str = "/gnss/septentrio/fix") -> List[Tuple[float, float, float, float]]:
        """
        Extract GPS messages from MCAP file.
        
        Args:
            bagfile: Path to MCAP file
            topic: GPS topic name
            
        Returns:
            List of tuples (timestamp, x, y, z) in TUM format
        """
        bagfile_path = Path(bagfile)
        if not bagfile_path.exists():
            raise FileNotFoundError(f"Bagfile not found: {bagfile}")
        
        print(f"Reading MCAP file: {bagfile}")
        print(f"Extracting topic: {topic}")
        
        trajectory = []
        message_count = 0
        skipped_invalid = 0
        found_topic = False
        
        with open(bagfile, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            
            for schema, channel, message, msg in reader.iter_decoded_messages(topics=[topic]):
                found_topic = True
                message_count += 1
                
                try:
                    # Extract timestamp (use message header timestamp)
                    msg_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    
                    lat = msg.latitude
                    lon = msg.longitude
                    alt = msg.altitude
                    
                    # Skip invalid GPS coordinates
                    if abs(lat) < 1e-6 and abs(lon) < 1e-6:
                        skipped_invalid += 1
                        continue
                    
                    # Initialize origin with first valid GPS point
                    if self.transformer is None:
                        self._initialize_origin(lat, lon, alt)
                    
                    # Convert to local coordinates
                    x, y, z = self._gps_to_local(lat, lon, alt)
                    
                    trajectory.append((msg_timestamp, x, y, z))
                    
                    # Progress indicator
                    if message_count % 100 == 0:
                        print(f"Processed {message_count} messages, extracted {len(trajectory)} valid points")
                        
                except Exception as e:
                    print(f"Warning: Failed to process message {message_count}: {e}", file=sys.stderr)
                    skipped_invalid += 1
                    continue
        
        if not found_topic:
            raise ValueError(f"Topic '{topic}' not found in bagfile. Use 'ros2 bag info' to list available topics.")
        
        print(f"\nExtraction complete:")
        print(f"  Total messages: {message_count}")
        print(f"  Valid GPS points: {len(trajectory)}")
        print(f"  Skipped invalid: {skipped_invalid}")
        
        return trajectory
    
    def write_tum_format(self, trajectory: List[Tuple[float, float, float, float]], output_file: str):
        """
        Write trajectory to TUM format file.
        
        Args:
            trajectory: List of (timestamp, x, y, z) tuples
            output_file: Output file path
        """
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        print(f"\nWriting trajectory to: {output_file}")
        
        with open(output_file, 'w') as f:
            for timestamp, x, y, z in trajectory:
                # TUM format: timestamp x y z qx qy qz qw
                # Using identity quaternion (0, 0, 0, 1) for GPS data (no orientation)
                f.write(f"{timestamp:.9f} {x:.6f} {y:.6f} {z:.6f} 0.0 0.0 0.0 1.0\n")
        
        print(f"Successfully wrote {len(trajectory)} poses to {output_file}")


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Extract GPS trajectory from MCAP bagfile and convert to TUM format.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Extract from default topic
  %(prog)s --bagfile /path/to/data.mcap --output reference.txt
  
  # Specify custom topic
  %(prog)s --bagfile data.mcap --topic /gnss/custom/fix --output ref.txt
        """
    )
    
    parser.add_argument(
        "--bagfile",
        type=str,
        required=True,
        help="Path to MCAP bagfile containing GPS messages"
    )
    
    parser.add_argument(
        "--topic",
        type=str,
        default="/gnss/septentrio/fix",
        help="GPS topic name (default: /gnss/septentrio/fix)"
    )
    
    parser.add_argument(
        "--output",
        type=str,
        default="reference_trajectory.txt",
        help="Output trajectory file path (default: reference_trajectory.txt)"
    )
    
    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_arguments()
    
    print("=" * 70)
    print("GPS Trajectory Extractor")
    print("=" * 70)
    
    try:
        # Create extractor
        extractor = GPSTrajectoryExtractor(use_first_as_origin=True)
        
        # Extract trajectory from MCAP
        trajectory = extractor.extract_from_mcap(args.bagfile, args.topic)
        
        if len(trajectory) == 0:
            print("\nError: No valid GPS points extracted!", file=sys.stderr)
            print(f"Check that topic '{args.topic}' exists and contains valid NavSatFix messages.", file=sys.stderr)
            sys.exit(1)
        
        # Write to TUM format
        extractor.write_tum_format(trajectory, args.output)
        
        print("\n" + "=" * 70)
        print("Extraction complete!")
        print("=" * 70)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

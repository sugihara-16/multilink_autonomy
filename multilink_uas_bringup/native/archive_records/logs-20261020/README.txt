Old native/logs archive, 2026-10-20

Drive folder: https://drive.google.com/drive/folders/1SNkp_XT7AsO2UyedbRQ1FqM1qJKQNiId
rclone destination: uas_gdrive_private:multilink_autonomy_archives/logs-20261020

Contents:
- logs-without-duplicated-bags.tar.zst.partNNNNN: one lossless zstd-compressed tar stream, split into 256 MiB parts. Concatenate in numeric order before decoding.
- archive_complete.json: all part sizes and MD5 values verified against Drive.
- existing_bags/: 47 MCAP files already stored on Drive, copied within Drive and verified against their local MD5 values. Not included again in the tar stream.
- existing_bags_verified.json: maps each original logs-relative path to its separately archived MCAP, size and MD5.
- inventory.json: original file tree, symlink targets, modes, times, inode/hardlink relationships.
- restore.py: restore into an EMPTY directory, checking downloaded archive parts and MCAP checksums.

Restore (requires rclone with the same authorized remote and zstd/tar):
  python3 restore.py --destination /path/to/empty/restore-directory

This recreates restore-directory/logs/. Use --without-bags to omit the 47 separately stored raw bags. Do not replace an existing logs directory containing newer experiments. Old absolute symlinks assume the original workspace paths; inspect them when restoring elsewhere.

New online simulations do NOT require restoring these logs. Inputs and calibration remain in native/simulation_inputs, documented in its README.md. Historical offline reprocessing and plot regeneration do require archived measurements.

#!/usr/bin/env python3
"""
PCD 点云地图高度过滤脚本
功能：读取 PCD 文件，自动识别地面，保留地面以上指定高度内的点云
用法：python3 filter_pcd_height.py <输入PCD路径> <输出PCD路径> [保留高度]
"""

import open3d as o3d
import numpy as np
import sys

def filter_pcd_by_height(input_pcd, output_pcd, keep_height=2.0):
    """
    根据点云高度分布自动过滤，保留地面以上 keep_height 米内的点云
    :param input_pcd: 输入PCD文件路径
    :param output_pcd: 输出PCD文件路径
    :param keep_height: 保留高度（米），默认2.0米
    """
    print(f"正在读取PCD文件: {input_pcd}")
    pcd = o3d.io.read_point_cloud(input_pcd)
    
    if not pcd.has_points():
        print("错误：点云为空或读取失败")
        return False
    
    points = np.asarray(pcd.points)
    print(f"原始点云数量: {len(points)}")
    
    # 提取Z轴数据
    z_values = points[:, 2]
    z_min = np.min(z_values)
    z_max = np.max(z_values)
    z_range = z_max - z_min
    
    print(f"点云Z轴范围: {z_min:.2f} ~ {z_max:.2f} 米 (总高 {z_range:.2f} 米)")
    
    # === 地面检测 ===
    # 方法：取最低的5%点云的平均高度作为地面
    sorted_z = np.sort(z_values)
    lowest_percentile = int(len(sorted_z) * 0.05)  # 取最低5%的点
    ground_z = np.mean(sorted_z[:lowest_percentile])
    
    # 如果地面检测结果异常（比如低于-10米），改用直方图方法
    if ground_z < -10 or ground_z > 10:
        print("地面检测异常，改用直方图方法...")
        hist, bin_edges = np.histogram(z_values, bins=100)
        # 找到前3个峰值中的最低峰值
        peak_indices = np.argsort(hist)[-3:]
        ground_bin = peak_indices[np.argmin(bin_edges[peak_indices])]
        ground_z = (bin_edges[ground_bin] + bin_edges[ground_bin + 1]) / 2
    
    print(f"检测到地面高度: {ground_z:.2f} 米")
    
    # === 计算过滤阈值 ===
    # 保留从地面到地面+keep_height之间的所有点
    floor_threshold = ground_z - 0.1  # 略微包含地面以下，避免丢失地面点
    ceiling_threshold = ground_z + keep_height
    
    print(f"过滤范围: {floor_threshold:.2f} ~ {ceiling_threshold:.2f} 米")
    print(f"保留地面以上 {keep_height} 米内的点云")
    
    # 执行过滤：保留在高度范围内的点
    mask = (points[:, 2] >= floor_threshold) & (points[:, 2] <= ceiling_threshold)
    filtered_points = points[mask]
    
    print(f"过滤后点云数量: {len(filtered_points)}")
    print(f"移除的点（天花板等）: {len(points) - len(filtered_points)} ({100*(1-len(filtered_points)/len(points)):.1f}%)")
    
    if len(filtered_points) == 0:
        print("错误：过滤后点云为空！")
        print("建议：增大 keep_height 参数")
        return False
    
    # 保存过滤后的点云
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    
    # 保留原始颜色信息（如果有）
    if pcd.has_colors():
        filtered_colors = np.asarray(pcd.colors)[mask]
        filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)
    
    # 保留原始法线信息（如果有）
    if pcd.has_normals():
        filtered_normals = np.asarray(pcd.normals)[mask]
        filtered_pcd.normals = o3d.utility.Vector3dVector(filtered_normals)
    
    o3d.io.write_point_cloud(output_pcd, filtered_pcd)
    print(f"过滤后的点云已保存到: {output_pcd}")
    
    # 输出统计信息
    print("\n=== 统计信息 ===")
    print(f"原始点数: {len(points)}")
    print(f"保留点数: {len(filtered_points)}")
    print(f"压缩率: {len(filtered_points)/len(points)*100:.1f}%")
    print(f"地面高度: {ground_z:.2f}m")
    print(f"保留高度: {keep_height}m")
    print(f"有效范围: {floor_threshold:.2f}m ~ {ceiling_threshold:.2f}m")
    
    return True


def main():
    # 用法说明
    if len(sys.argv) < 3:
        print("用法: python3 filter_pcd_height.py <输入PCD路径> <输出PCD路径> [保留高度]")
        print("示例: python3 filter_pcd_height.py map.pcd filtered_map.pcd")
        print("示例: python3 filter_pcd_height.py map.pcd filtered_map.pcd 2.5")
        print("  - 保留高度默认2.0米，表示保留地面以上2米范围内的点")
        print("  - 这样会移除天花板和高于2米的障碍物")
        sys.exit(1)
    
    input_pcd = sys.argv[1]
    output_pcd = sys.argv[2]
    
    # 默认保留高度2米
    keep_height = 2.0
    if len(sys.argv) >= 4:
        keep_height = float(sys.argv[3])
        if keep_height <= 0:
            print("错误：保留高度必须大于0")
            sys.exit(1)
    
    success = filter_pcd_by_height(input_pcd, output_pcd, keep_height)
    
    if success:
        print("\n✓ 过滤完成！")
    else:
        print("\n✗ 过滤失败！")
        sys.exit(1)


if __name__ == "__main__":
    main()

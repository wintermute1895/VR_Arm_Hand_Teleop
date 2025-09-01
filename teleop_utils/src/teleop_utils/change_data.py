# 读取文件内容
with open('move_data.txt', 'r') as file:
    lines = file.readlines()

# 处理每一行数据
modified_lines = []
for line in lines:
    # 分割制表符数据并转换为整数
    data = line.strip().split('\t')
    data = [int(x) for x in data]
    
    # 修改指定列 (列索引从0开始计算)
    data[0] = 200    # 第1列 → 索引0
    data[5] = 200    # 第6列 → 索引5
    data[10] = 128   # 第11列 → 索引10
    data[11] = 200   # 第12列 → 索引11
    
    # 转换回字符串格式
    modified_lines.append("\t".join(map(str, data)))

# 写入新文件
with open('move_data_modified.txt', 'w') as file:
    file.write("\n".join(modified_lines))

print("文件修改完成，已保存为 move_data_modified.txt")
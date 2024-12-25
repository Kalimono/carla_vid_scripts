import os

def check_and_delete_files(folder1, folder2, folder3):
    # Get sets of filenames in each folder
    files1 = set(os.listdir(folder1))
    files2 = set(os.listdir(folder2))
    files3 = set(os.listdir(folder3))
    
    # Find files that are not common to all three folders
    all_files = files1.union(files2).union(files3)
    common_files = files1.intersection(files2).intersection(files3)
    files_to_delete = all_files - common_files
    
    # Delete files that are not common in each folder
    for folder in [folder1, folder2, folder3]:
        for file in files_to_delete:
            file_path = os.path.join(folder, file)
            if os.path.exists(file_path):
                os.remove(file_path)
                print(f"Deleted {file_path}")

# Example usage
folder1 = "left_mirror"
folder2 = "right_mirror"
folder3 = "windshield"

check_and_delete_files(folder1, folder2, folder3)

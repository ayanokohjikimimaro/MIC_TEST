import struct
import numpy as np
import matplotlib.pyplot as plt
import os
import serial # ★ pySerialライブラリをインポート
import time   # ★ timeライブラリを追加 (任意で待機などに使用)
import datetime

# --- 設定 ---
# ★ シリアルポート設定 (環境に合わせて変更してください)
SERIAL_PORT = 'COM4'  # Windowsの場合 'COMx' (例: 'COM3'), Linuxの場合 '/dev/ttyACMx' や '/dev/ttyUSBx'
BAUD_RATE = 115200

# --- データ設定 (変更なし) ---
NUM_SAMPLES = 8000
BYTES_PER_SAMPLE = 4
TOTAL_BINARY_BYTES = NUM_SAMPLES * BYTES_PER_SAMPLE
SAMPLING_FREQUENCY = 16000
START_MARKER_BASE = b"Sending Buffer Half" # 開始マーカー
MARKER_END = b"\r\n"                # マーカー後の改行
END_MARKER_BASE = b"Buffer Half"        # 終了マーカーの開始部分
# END_MARKER_BASE = b"Sent.\r\n"        # 終了マーカーの開始部分
STRUCT_FORMAT = f'<{NUM_SAMPLES}i'       # unpackフォーマット


# --- ★ ファイル保存設定 ---
SAVE_DIRECTORY = "audio_data" # 保存先フォルダ (スクリプトと同じ場所からの相対パス or 絶対パス)
FILENAME_PREFIX = "audio"     # ファイル名の接頭辞

def read_serial_and_plot_audio(port, baudrate, timeout_sec=10, SAVE_EN=False):
    """
    指定されたシリアルポートからオーディオデータを受信し、プロットする関数
    """
    ser = None # シリアルポートオブジェクト初期化
    try:
        # --- ★ シリアルポートを開く ---
        print(f"シリアルポート {port} を開いています (Baud: {baudrate})...")
        ser = serial.Serial(port, baudrate, timeout=timeout_sec)
        print("ポートを開きました。")
        print("STM32ボードのボタンを押してデータ送信を開始してください...")
        
        # ★★★ 追加: 入力バッファをクリア ★★★
        try:
            ser.reset_input_buffer()
            print("情報: シリアル入力バッファをクリアしました。")
        except Exception as e:
            print(f"警告: 入力バッファクリア中にエラー: {e}")        

        # --- ★ 開始マーカーを待つ ---
        start_marker_found = False
        received_line = b""
        while not start_marker_found:
            try:
                received_line = ser.readline() # 1行読み込み (改行コードまで)
                print(f"受信(待機中): {received_line.decode('ascii', errors='ignore').strip()}") # デバッグ用
                if START_MARKER_BASE in received_line:
                    start_marker_found = True
                    print("開始マーカーを検出しました。バイナリデータ受信開始...")
            except serial.SerialTimeoutException:
                print(f"エラー: 開始マーカー受信中にタイムアウト ({timeout_sec}秒) しました。")
                return
            except Exception as e:
                print(f"エラー: シリアル受信中にエラーが発生しました: {e}")
                return

        # --- ★ バイナリデータを読み込む ---
        try:
            # read()で指定バイト数を読み込む
            binary_data = ser.read(TOTAL_BINARY_BYTES)
        except Exception as e:
            print(f"エラー: バイナリデータ受信中にエラーが発生しました: {e}")
            return

        actual_binary_bytes = len(binary_data)
        print(f"情報: 受信したバイナリデータの長さ: {actual_binary_bytes} バイト")

        if actual_binary_bytes < TOTAL_BINARY_BYTES:
            print(f"エラー: 期待されるバイナリデータ長 ({TOTAL_BINARY_BYTES} バイト) を受信できませんでした。")
            # 不完全なデータでも処理を続ける場合は以下のコメントアウトを解除
            # print("警告: 不完全なデータで処理を続行します。")
            # num_actual_samples = actual_binary_bytes // BYTES_PER_SAMPLE
            # bytes_to_unpack = num_actual_samples * BYTES_PER_SAMPLE
            # if bytes_to_unpack == 0:
            #     print("エラー: 読み込める完全なサンプルがありません。")
            #     return
            # current_struct_format = f'<{num_actual_samples}i'
            # binary_data = binary_data[:bytes_to_unpack] # 不完全な部分を切り捨て
            return # ここではエラーとして終了する

        elif actual_binary_bytes > TOTAL_BINARY_BYTES:
             print("警告: 期待より多くのバイナリデータを受信しました。期待される長さで処理します。")
             binary_data = binary_data[:TOTAL_BINARY_BYTES]
             num_actual_samples = NUM_SAMPLES
             current_struct_format = STRUCT_FORMAT
        else:
            # 期待通りの長さ
            num_actual_samples = NUM_SAMPLES
            current_struct_format = STRUCT_FORMAT

            if SAVE_EN:
                # --- ★ ファイル保存処理 ---
                try:
                    # 保存先ディレクトリがなければ作成
                    if not os.path.exists(SAVE_DIRECTORY):
                        os.makedirs(SAVE_DIRECTORY)

                    # タイムスタンプ付きファイル名を生成 (例: audio_20250503_143055.bin)
                    now = datetime.datetime.now()
                    timestamp_str = now.strftime("%Y%m%d_%H%M%S")
                    save_filename = os.path.join(SAVE_DIRECTORY, f"{FILENAME_PREFIX}_{timestamp_str}.bin")

                    # バイナリ書き込みモードでファイルを開いて書き込む
                    with open(save_filename, 'wb') as f:
                        f.write(binary_data)
                    print(f"バイナリデータを {save_filename} に保存しました。")

                except Exception as e:
                    print(f"エラー: ファイル '{save_filename}' の保存中にエラーが発生しました: {e}")
                    # 保存エラーでもプロットは試みる場合あり

        # --- ★ 終了マーカーを読み込む (任意だが推奨) ---
        try:
            # 終了マーカーを含む可能性のある行を読む
            # タイムアウトを短めに設定しても良い
            ser.timeout = 1 # 例: 1秒
            end_line = ser.readline()
            print(f"受信(終了確認): {end_line.decode('ascii', errors='ignore').strip()}")
            if END_MARKER_BASE not in end_line:
                 print("警告: 期待される終了マーカーが見つかりませんでした。")
        except serial.SerialTimeoutException:
            print(f"警告: 終了マーカー受信中にタイムアウトしました。")
        except Exception as e:
            print(f"警告: 終了マーカー受信中にエラーが発生しました: {e}")


        # --- バイナリデータを数値に変換 ---
        try:
            unpacked_data = struct.unpack(current_struct_format, binary_data)
        except struct.error as e:
            print(f"エラー: バイナリデータの展開に失敗しました: {e}")
            return

        audio_samples = np.array(unpacked_data)
        print(f"DEBUG: Last 5 samples: {audio_samples[-5:]}")
        time_axis = np.arange(num_actual_samples) / SAMPLING_FREQUENCY

        # --- グラフ描画 ---
        if False:
            plt.figure(figsize=(12, 6))
            plt.subplot(2, 1, 1)
            plt.suptitle(f'Audio Waveform from {port} ({num_actual_samples} samples)')

            plt.plot(time_axis, audio_samples)
            # plt.title(f'Audio Waveform from {port} ({num_actual_samples} samples)')
            plt.xlabel('Time (sec)')
            plt.ylabel('Amplitude (int32)')
            plt.grid(True)

            plt.subplot(2, 1, 2)
            plt.plot(time_axis[0:50]*1000, audio_samples[0:50], marker="o", markersize=2)
            # plt.title(f'Audio Waveform from {port} ({num_actual_samples} samples)')
            plt.xlabel('Time (ms)')
            plt.ylabel('Amplitude (int32)')
            plt.grid(True)

            plt.tight_layout()
            plt.show()

        plt.figure(figsize=(12, 6))
        plt.title(f'Audio Waveform from {port} ({num_actual_samples} samples)')

        plt.plot(time_axis, audio_samples)
        # plt.title(f'Audio Waveform from {port} ({num_actual_samples} samples)')
        plt.xlabel('Time (sec)')
        plt.ylabel('Amplitude (int32)')
        plt.grid(True)

        plt.tight_layout()
        plt.show()

        print(f"グラフのプロットが完了しました。")
        print(f"サンプル数: {len(audio_samples)}")
        if len(audio_samples) > 0:
             print(f"最小値: {np.min(audio_samples)}")
             print(f"最大値: {np.max(audio_samples)}")
             print(f"平均値: {np.mean(audio_samples):.2f}")

    except serial.SerialException as e:
        print(f"エラー: シリアルポートを開けません: {e}")
        print("正しいCOMポート名か、他のプログラムが使用中でないか確認してください。")
    except Exception as e:
        print(f"予期せぬエラーが発生しました: {e}")
    finally:
        # --- ★ シリアルポートを閉じる ---
        if ser and ser.is_open:
            ser.close()
            print(f"シリアルポート {port} を閉じました。")


if __name__ == "__main__":
    # ★ 実行前に SERIAL_PORT を正しい値に設定してください ★
    read_serial_and_plot_audio(SERIAL_PORT, BAUD_RATE, SAVE_EN=False)
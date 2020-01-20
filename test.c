int main() {
	int text = 0xFF0000;
	printf("%d\n", (text >> 16) & 0x7F);
}

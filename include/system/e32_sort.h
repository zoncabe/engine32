/*
	The engine's sort. There is no libc underneath, so this stands in for
	qsort, with the same algorithm newlib runs: the BSD quicksort, pivot by
	median of three (of nine past 40 items), insertion sort under 7 items,
	recursion only into the smaller half so the stack stays logarithmic.

	A template instead of qsort's void pointers: it compares and copies
	by type, without a function pointer per compare or a byte swap.

	    sort(items, count, [](const T &a, const T &b) { return a < b; });
*/
#ifndef ENGINE_32_SORT_H
#define ENGINE_32_SORT_H


template <typename T>
static inline void sort_swap(T &a, T &b)
{
	T t = a;
	a = b;
	b = t;
}

template <typename T, typename Less>
static inline T *sort_median3(T *a, T *b, T *c, Less less)
{
	return less(*a, *b)
		? (less(*b, *c) ? b : (less(*a, *c) ? c : a))
		: (less(*c, *b) ? b : (less(*c, *a) ? c : a));
}

template <typename T, typename Less>
void sort(T *items, int count, Less less)
{
	for (;;) {
		if (count < 7) {
			for (T *i = items + 1; i < items + count; i++)
				for (T *j = i; j > items && less(*j, *(j - 1)); j--)
					sort_swap(*j, *(j - 1));
			return;
		}

		/* The pivot: the middle item, or the median of three, or of nine
		   for larger runs. Then it goes to the front. */
		T *mid = items + count / 2;
		if (count > 7) {
			T *lo = items;
			T *hi = items + count - 1;
			if (count > 40) {
				int d = count / 8;
				lo  = sort_median3(lo,      lo + d,  lo + 2 * d, less);
				mid = sort_median3(mid - d, mid,     mid + d,    less);
				hi  = sort_median3(hi - 2 * d, hi - d, hi,       less);
			}
			mid = sort_median3(lo, mid, hi, less);
		}
		sort_swap(*items, *mid);

		/* Partition: items equal to the pivot collect at both ends and are
		   swapped into the middle afterwards, so runs of equal keys do not
		   degrade it. */
		T *pa = items + 1, *pb = pa;
		T *pc = items + count - 1, *pd = pc;

		for (;;) {
			while (pb <= pc && !less(*items, *pb)) {
				if (!less(*pb, *items)) { sort_swap(*pa, *pb); pa++; }
				pb++;
			}
			while (pb <= pc && !less(*pc, *items)) {
				if (!less(*items, *pc)) { sort_swap(*pc, *pd); pd--; }
				pc--;
			}
			if (pb > pc) break;
			sort_swap(*pb, *pc);
			pb++;
			pc--;
		}

		T *end = items + count;
		int r;

		r = (pa - items) < (pb - pa) ? (pa - items) : (pb - pa);
		for (T *x = items, *y = pb - r; r > 0; r--) sort_swap(*x++, *y++);

		r = (pd - pc) < (end - pd - 1) ? (pd - pc) : (end - pd - 1);
		for (T *x = pb, *y = end - r; r > 0; r--) sort_swap(*x++, *y++);

		int left  = pb - pa;
		int right = pd - pc;

		/* Recurse into the smaller side, loop on the larger. */
		if (left < right) {
			if (left > 1) sort(items, left, less);
			items = end - right;
			count = right;
		} else {
			if (right > 1) sort(end - right, right, less);
			count = left;
		}
		if (count <= 1) return;
	}
}


#endif
